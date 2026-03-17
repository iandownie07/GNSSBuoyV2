#include "Common/messaging.h"

#define QUEUE_SIZE 10

// Queue definition
TX_QUEUE gnss_to_ble_queue;
TX_QUEUE gnss_to_ekf_queue;
static ULONG queue_buffer[QUEUE_SIZE * sizeof(gnss_velocity_msg_t) / sizeof(ULONG)];

// Event flags definition
TX_EVENT_FLAGS_GROUP gnss_events;
TX_EVENT_FLAGS_GROUP ekf_events;

void messaging_init(void) {
    UINT status;
    
    // Create the message queue
    status = tx_queue_create(&gnss_to_ble_queue, 
                            "GNSS to BLE Queue",
                            sizeof(gnss_velocity_msg_t) / sizeof(ULONG),
                            queue_buffer,
                            sizeof(queue_buffer));
    
    if (status != TX_SUCCESS) {
        // Handle error - maybe toggle an LED or log
        printf("Problem with BLE queue\n");
        while(1); // Fatal error
    }
    
    // Create the event flags group
    status = tx_event_flags_create(&gnss_events, "GNSS Events");
    
    if (status != TX_SUCCESS) {
        // Handle error
        printf("Problem with BLE queue\n");
        while(1); // Fatal error
    }

        // Create the message queue
    status = tx_queue_create(&gnss_to_ekf_queue, 
                            "GNSS to EKF Queue",
                            sizeof(gnss_to_ekf_msg_t) / sizeof(ULONG),
                            queue_buffer,
                            sizeof(queue_buffer));
    
    if (status != TX_SUCCESS) {
        // Handle error - maybe toggle an LED or log
        printf("Problem with EKF queue\n");
        while(1); // Fatal error
    }
    
    // Create the event flags group
    status = tx_event_flags_create(&ekf_events, "EKF Events");
    
    if (status != TX_SUCCESS) {
        // Handle error
        printf("Problem with EKF queue\n");
        while(1); // Fatal error
    }
}