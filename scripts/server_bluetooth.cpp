#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <vector>
#include <string>
#include <cstring>

// Function prototype for connecting to a device
int connectToDevice(int sock, bdaddr_t *bdaddr);

int main() {
    // Initialize the Bluetooth adapter
    int dev_id = hci_get_route(NULL);
    // Open a socket to the Bluetooth adapter
    int sock = hci_open_dev(dev_id);
    if (dev_id < 0 || sock < 0) {
        // Handle error: unable to access the Bluetooth adapter
        return 1;
    }

    // Set up scan parameters
    uint8_t scan_type = 0x01;  // Active scanning (0x00 for passive)
    uint16_t interval = htobs(0x0010);  // Scan interval
    uint16_t window = htobs(0x0010);  // Scan window
    uint8_t own_type = LE_PUBLIC_ADDRESS;  // Use public address
    uint8_t filter_policy = 0x00;  // No whitelist

    // Apply scan parameters to the Bluetooth adapter
    if (hci_le_set_scan_parameters(sock, scan_type, interval, window, own_type, filter_policy, 1000) < 0) {
        // Handle error: unable to set scan parameters
        return 1;
    }

    // Enable scanning
    uint8_t filter_dup = 0;  // Do not filter duplicate results
    if (hci_le_set_scan_enable(sock, 0x01, filter_dup, 1000) < 0) {
        // Handle error: unable to start scanning
        return 1;
    }

    // Set up HCI filter to receive only LE meta events
    struct hci_filter nf, of;
    socklen_t olen = sizeof(of);
    if (getsockopt(sock, SOL_HCI, HCI_FILTER, &of, &olen) < 0) {
        // Handle error: unable to get socket options
        return 1;
    }

    hci_filter_clear(&nf);
    hci_filter_set_ptype(HCI_EVENT_PKT, &nf);
    hci_filter_set_event(EVT_LE_META_EVENT, &nf);

    if (setsockopt(sock, SOL_HCI, HCI_FILTER, &nf, sizeof(nf)) < 0) {
        // Handle error: unable to set socket options
        return 1;
    }

    // List of target device addresses 
    // that address right there is joe's laptop 
    std::vector<std::string> targetDevices = {"4C:79:6E:DB:1B:17"};
    std::vector<std::string> connectedDevices;

    // Buffer to store incoming data
    unsigned char buf[HCI_MAX_EVENT_SIZE];
    
    // Main scanning loop
    while (1) {
        // Read incoming Bluetooth packets
        int len = read(sock, buf, sizeof(buf));
        if (len < 0) {
            // Handle error: unable to read from socket
            break;
        }

        // Parse the event
        evt_le_meta_event *meta = (evt_le_meta_event*)(buf + HCI_EVENT_HDR_SIZE + 1);
        if (meta->subevent != EVT_LE_ADVERTISING_REPORT) {
            continue;  // Skip non-advertising events
        }

        // Extract advertising information
        le_advertising_info *info = (le_advertising_info *)(meta->data + 1);
        char addr[18];
        ba2str(&info->bdaddr, addr);  // Convert Bluetooth address to string

        // Check if the discovered device is in our target list
        for (const auto& targetAddr : targetDevices) {
            if (strcmp(addr, targetAddr.c_str()) == 0) {
                printf("Found target device: %s\n", addr);
                // Attempt to connect to the device
                if (connectToDevice(sock, &info->bdaddr) == 0) {
                    printf("Connected to device %s\n", addr);
                    connectedDevices.push_back(targetAddr);
                    // Remove from target list or mark as connected
                }
                break;  // Move to next device in scan results
            }
        }

        // Check if all devices are connected
        if (connectedDevices.size() == targetDevices.size()) {
            // All devices connected, stop scanning
            break;
        }
    }

    // Disable scanning
    hci_le_set_scan_enable(sock, 0x00, filter_dup, 1000);

    // TODO add event loop here?
  //
  if (!connectedDevices.empty()) {
      // Event loop for processing commands
      bool running = true;
      while (running) {
          for (const auto& deviceAddr : connectedDevices) {
              // For each connected device
              int bytes_read;
              char buffer[1024] = { 0 };

              // Read data from the device
              bytes_read = read(sock, buffer, sizeof(buffer));
              if (bytes_read > 0) {
                  // Process the received data
                  processCommand(buffer, bytes_read, deviceAddr);
              } else if (bytes_read < 0) {
                  // Handle read error
                  printf("Error reading from device %s\n", deviceAddr.c_str());
                  running = false;
                  break;
              }

              // Add any other necessary checks or operations here
          }

          // Add a small delay to avoid busy-waiting
          usleep(10000);  // Sleep for 10ms
      }
  }

    // Close the Bluetooth socket
    close(sock);

    return 0;
}

// Function to connect to a Bluetooth device
int connectToDevice(int sock, bdaddr_t *bdaddr) {
    // Allocate memory for connection info request
    struct hci_conn_info_req *cr = (struct hci_conn_info_req*)malloc(sizeof(*cr) + sizeof(struct hci_conn_info));
    bacpy(&cr->bdaddr, bdaddr);  // Copy Bluetooth address
    cr->type = ACL_LINK;  // Set connection type to ACL

    // Check if device is already connected
    if (ioctl(sock, HCIGETCONNINFO, (unsigned long) cr) < 0) {
        // Device is not connected, so attempt to connect
        if (hci_create_connection(sock, bdaddr, htobs(0x0008), 0, 0, NULL, 0) < 0) {
            free(cr);
            return -1;  // Connection failed
        }
    }
    free(cr);
    return 0;  // Connection successful or already connected
}


void processCommand(char* buffer, int length, const std::string& deviceAddr) {
    // Implement your command processing logic here
    printf("Received %d bytes from device %s: %s\n", length, deviceAddr.c_str(), buffer);
    // Add your command interpretation and execution logic
  
  // format buffer as string

  // strip unused bytes (zero bytes)
  
  // return string


  
}



