#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "oc_protocol.pio.h" // Generated header from the .pio files

// --- Configuration ---
#define OC_PIN 22          // Shared Open-Collector I/O Pin
#define OC_PIN_DIR 26      // A second pin for the PINDIRS side-set, for simplicity (optional, can be same as OC_PIN)
#define PROTOCOL_PIO pio0
#define RX_SM 0
#define TX_SM 1
#define BYTES_PER_TRANSFER 8
#define CLK_DIV 62.5f      // 125MHz / (1MHz * 2 cycles per bit) = 62.5

// --- Buffers ---
uint8_t rx_buffer[BYTES_PER_TRANSFER];
uint8_t tx_buffer[BYTES_PER_TRANSFER] = {0xAA, 0xBB, 0xCC, 0xDD, 0x11, 0x22, 0x33, 0x44}; // Example response data

// --- DMA Channels ---
int rx_dma_chan;
int tx_dma_chan;

// --- Forward Declarations ---
void oc_protocol_init(PIO pio, uint sm_rx, uint sm_tx, uint pin);
void oc_rx_dma_handler();

// --- Main Program ---
int main() {
    stdio_init_all();
    
    // Set up the protocol
    oc_protocol_init(PROTOCOL_PIO, RX_SM, TX_SM, OC_PIN);

    printf("Starting Open-Collector Protocol Handler...\n");
    printf("Waiting for 8 bytes on GPIO %d...\n", OC_PIN);

    // Start the RX DMA transfer (the SM is already running)
    dma_channel_set_trans_count(rx_dma_chan, BYTES_PER_TRANSFER, true);

    while (true) {
        tight_loop_contents();
    }
}

// --- Initialization ---

void oc_protocol_init(PIO pio, uint sm_rx, uint sm_tx, uint pin) {
    uint rx_offset = pio_add_program(pio, &oc_rx_program);
    uint tx_offset = pio_add_program(pio, &oc_tx_program);

    // 1. Initialize PIO State Machines
    
    // RX SM (SM0)
    pio_sm_config c_rx = oc_rx_program_get_default_config(rx_offset);
    sm_config_set_in_pins(&c_rx, pin);
    sm_config_set_clkdiv(&c_rx, CLK_DIV);
    sm_config_set_in_shift(&c_rx, true, true, 8); // Auto-push after 8 bits, LSB first
    pio_sm_init(pio, sm_rx, rx_offset, &c_rx);
    pio_sm_set_enabled(pio, sm_rx, true);

    // TX SM (SM1)
    pio_sm_config c_tx = oc_tx_program_get_default_config(tx_offset);
    sm_config_set_out_pins(&c_tx, pin, 1);
    sm_config_set_sideset_pins(&c_tx, pin); // Use pin for the side-set to control the line state
    sm_config_set_clkdiv(&c_tx, CLK_DIV);
    sm_config_set_out_shift(&c_tx, true, true, 8); // Auto-pull when FIFO is empty, LSB first
    // Note: The TX SM is disabled initially, will be enabled in the handler.
    pio_sm_init(pio, sm_tx, tx_offset, &c_tx);
    pio_sm_set_enabled(pio, sm_tx, false); 
    
    // 2. Initialize GPIO
    pio_gpio_init(pio, pin); // Set the pin for PIO control
    gpio_set_pulls(pin, true, false); // Enable pull-up for open-collector idle state (high)

    // 3. Setup DMA for RX
    rx_dma_chan = dma_claim_unused_channel(true);
    dma_channel_config c_rx_dma = dma_channel_get_default_config(rx_dma_chan);
    channel_config_set_read_increment(&c_rx_dma, false); // Read from PIO FIFO (fixed address)
    channel_config_set_write_increment(&c_rx_dma, true);  // Write to RAM buffer (increment address)
    channel_config_set_dreq(&c_rx_dma, pio_get_dreq(pio, sm_rx, false)); // DREQ from RX FIFO
    channel_config_set_transfer_data_size(&c_rx_dma, DMA_SIZE_BYTE); // 8-bit transfers

    dma_channel_configure(
        rx_dma_chan,
        &c_rx_dma,
        rx_buffer,                  // Destination: RAM buffer
        &pio->rxf[sm_rx],           // Source: PIO RX FIFO
        BYTES_PER_TRANSFER,         // Transfer count (for 1 byte)
        false                       // Don't start immediately
    );

    // Configure DMA interrupt for RX completion
    dma_channel_set_irq0_enabled(rx_dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, oc_rx_dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    // 4. Setup DMA for TX (Done in handler)
    tx_dma_chan = dma_claim_unused_channel(true);
    dma_channel_config c_tx_dma = dma_channel_get_default_config(tx_dma_chan);
    channel_config_set_read_increment(&c_tx_dma, true);  // Read from RAM buffer (increment address)
    channel_config_set_write_increment(&c_tx_dma, false); // Write to PIO FIFO (fixed address)
    channel_config_set_dreq(&c_tx_dma, pio_get_dreq(pio, sm_tx, true)); // DREQ from TX FIFO
    channel_config_set_transfer_data_size(&c_tx_dma, DMA_SIZE_BYTE); // 8-bit transfers
    
    dma_channel_configure(
        tx_dma_chan,
        &c_tx_dma,
        &pio->txf[sm_tx],           // Destination: PIO TX FIFO
        tx_buffer,                  // Source: RAM buffer
        BYTES_PER_TRANSFER,         // Transfer count (for 1 byte)
        false                       // Don't start immediately
    );
    // Note: No interrupt for TX, as we rely on the SM running out to know it's done.
}

// --- Interrupt Handler for RX DMA Completion ---

void oc_rx_dma_handler() {
    if (dma_channel_get_irq0_status(rx_dma_chan)) {
        dma_channel_acknowledge_irq0(rx_dma_chan);

        // A. Process received data
        printf("\nRX Complete. Received data:\n");
        for (int i = 0; i < BYTES_PER_TRANSFER; i++) {
            printf("0x%02X ", rx_buffer[i]);
            // In a real application, you would calculate the response here, e.g.:
            // tx_buffer[i] = rx_buffer[i] + 1; 
        }
        printf("\n\nStarting TX with data:\n");
        for (int i = 0; i < BYTES_PER_TRANSFER; i++) {
            printf("0x%02X ", tx_buffer[i]);
        }
        printf("\n");

        // B. Start TX sequence

        // 1. Disable RX SM and clear its FIFO (if necessary)
        pio_sm_set_enabled(PROTOCOL_PIO, RX_SM, false);
        pio_sm_clear_fifos(PROTOCOL_PIO, RX_SM);

        // 2. Enable TX SM (it will automatically start pulling data)
        pio_sm_set_enabled(PROTOCOL_PIO, TX_SM, true);
        
        // 3. Reconfigure and trigger TX DMA
        dma_channel_set_trans_count(tx_dma_chan, BYTES_PER_TRANSFER, false); // Set count
        dma_channel_set_read_addr(tx_dma_chan, tx_buffer, true); // Set source address and start

        // 4. Wait for TX DMA to finish (optional, for blocking communication)
        dma_channel_wait_for_finish_blocking(tx_dma_chan);
        
        // 5. Disable TX SM and clear its FIFO
        pio_sm_set_enabled(PROTOCOL_PIO, TX_SM, false);
        pio_sm_clear_fifos(PROTOCOL_PIO, TX_SM);

        // C. Re-enable RX for the next packet
        printf("TX Complete. Re-enabling RX.\n");
        pio_sm_set_enabled(PROTOCOL_PIO, RX_SM, true);
        dma_channel_set_trans_count(rx_dma_chan, BYTES_PER_TRANSFER, true); // Re-arm RX DMA
    }
}