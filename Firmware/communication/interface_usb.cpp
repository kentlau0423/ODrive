
#include "interface_usb.h"
#include "ascii_protocol.hpp"

#include <MotorControl/utils.hpp>

#include <fibre/protocol.hpp>
#include <fibre/../../async_stream.hpp>
#include <fibre/../../legacy_protocol.hpp>
#include <usbd_cdc.h>
#include <usbd_cdc_if.h>
#include <usb_device.h>
#include <cmsis_os.h>
#include <freertos_vars.h>

#include <odrive_main.h>

osThreadId usb_thread;
const uint32_t stack_size_usb_thread = 4096; // Bytes
USBStats_t usb_stats_;

namespace fibre {

class Stm32UsbTxStream : public AsyncStreamSink {
public:
    Stm32UsbTxStream(uint8_t endpoint_num) : endpoint_num_(endpoint_num) {}

    TransferHandle start_write(cbufptr_t buffer, Completer<WriteResult>& completer) final;
    void cancel_write(TransferHandle transfer_handle) final;
    void did_finish();

    const uint8_t endpoint_num_;
    bool connected_ = false;
    Completer<WriteResult>* completer_ = nullptr;
    const uint8_t* tx_end_ = nullptr;
};

class Stm32UsbRxStream : public AsyncStreamSource {
public:
    Stm32UsbRxStream(uint8_t endpoint_num) : endpoint_num_(endpoint_num) {}

    TransferHandle start_read(bufptr_t buffer, Completer<ReadResult>& completer) final;
    void cancel_read(TransferHandle transfer_handle) final;
    void did_finish();
    
    const uint8_t endpoint_num_;
    bool connected_ = false;
    Completer<ReadResult>* completer_ = nullptr;
    uint8_t* rx_end_ = nullptr;
};

}

using namespace fibre;

TransferHandle Stm32UsbTxStream::start_write(cbufptr_t buffer, Completer<WriteResult>& completer) {
    if (!connected_) {
        completer.complete({kStreamClosed, buffer.begin()});
        return 0;
    }

    if (buffer.size() > USB_TX_DATA_SIZE) {
        completer.complete({kStreamError, buffer.begin()});
        return 0;
    }

    if (completer_ || tx_end_) {
        completer.complete({kStreamError, buffer.begin()});
        return 0;
    }

    completer_ = &completer;
    tx_end_ = buffer.end();

    if (CDC_Transmit_FS(const_cast<uint8_t*>(buffer.begin()), buffer.size(), endpoint_num_) != USBD_OK) {
        tx_end_ = nullptr;
        safe_complete(completer_, {kStreamError, buffer.begin()});
        return 0;
    }

    return reinterpret_cast<TransferHandle>(this);
}

void Stm32UsbTxStream::cancel_write(TransferHandle transfer_handle) {
    // not implemented
}

void Stm32UsbTxStream::did_finish() {
    const uint8_t* tx_end = tx_end_;
    tx_end_ = nullptr;
    safe_complete(completer_, {connected_ ? kStreamOk : kStreamClosed, tx_end});
}

TransferHandle Stm32UsbRxStream::start_read(bufptr_t buffer, Completer<ReadResult>& completer) {
    if (!connected_) {
        completer.complete({kStreamClosed, buffer.begin()});
        return 0;
    }

    if (completer_ || rx_end_) {
        completer.complete({kStreamError, buffer.begin()});
        return 0;
    }

    completer_ = &completer;
    rx_end_ = buffer.begin(); // the pointer is updated at the end of the transfer

    if (USBD_CDC_ReceivePacket(&hUsbDeviceFS, buffer.begin(), buffer.size(), endpoint_num_) != USBD_OK) {
        rx_end_ = nullptr;
        safe_complete(completer_, {kStreamError, buffer.begin()});
        return 0;
    }

    return reinterpret_cast<TransferHandle>(this);
}

void Stm32UsbRxStream::cancel_read(TransferHandle transfer_handle) {
    // not implemented
}

void Stm32UsbRxStream::did_finish() {
    uint8_t* rx_end = rx_end_;
    rx_end_ = nullptr;
    safe_complete(completer_, {connected_ ? kStreamOk : kStreamClosed, rx_end});
}

Stm32UsbTxStream usb_cdc_tx_stream(CDC_IN_EP);
Stm32UsbTxStream usb_native_tx_stream(ODRIVE_IN_EP);
Stm32UsbRxStream usb_cdc_rx_stream(CDC_OUT_EP);
Stm32UsbRxStream usb_native_rx_stream(ODRIVE_OUT_EP);

//// This is used by the printf feature. Hence the above statics, and below seemingly random ptr (it's externed)
//// TODO: fix printf
//StreamSink* usb_stream_output_ptr = &usb_stream_output;

AsciiProtocol ascii_over_cdc(&usb_cdc_rx_stream, &usb_cdc_tx_stream);
LegacyProtocolStreamBased fibre_over_cdc(&usb_cdc_rx_stream, &usb_cdc_tx_stream);
LegacyProtocolPacketBased fibre_over_usb(&usb_native_rx_stream, &usb_native_tx_stream, USB_TX_DATA_SIZE);

static void usb_server_thread(void * ctx) {
    (void) ctx;
 
    for (;;) {
        osEvent event = osMessageGet(usb_event_queue, osWaitForever);

        if (event.status != osEventMessage) {
            continue;
        }

        usb_stats_.rx_cnt++;

        switch (event.value.v) {
            case 1: { // USB connected event
                usb_cdc_tx_stream.connected_ = true;
                usb_native_tx_stream.connected_ = true;
                usb_cdc_rx_stream.connected_ = true;
                usb_native_rx_stream.connected_ = true;

                fibre_over_usb.start(Completer<LegacyProtocolPacketBased*, StreamStatus>::get_dummy());

                if (odrv.config_.enable_ascii_protocol_on_usb) {
                    ascii_over_cdc.start();
                } else {
                    fibre_over_cdc.start(Completer<LegacyProtocolPacketBased*, StreamStatus>::get_dummy());
                }
            } break;

            case 2: { // USB disconnected event
                usb_cdc_tx_stream.connected_ = false;
                usb_native_tx_stream.connected_ = false;
                usb_cdc_rx_stream.connected_ = false;
                usb_native_rx_stream.connected_ = false;
                usb_cdc_tx_stream.did_finish();
                usb_native_tx_stream.did_finish();
                usb_cdc_rx_stream.did_finish();
                usb_native_rx_stream.did_finish();
            } break;

            case 3: { // TX on CDC interface done
                usb_cdc_tx_stream.did_finish();
            } break;

            case 4: { // TX on custom interface done
                usb_native_tx_stream.did_finish();
            } break;

            case 5: { // RX on CDC interface done
                usb_cdc_rx_stream.did_finish();
            } break;

            case 6: { // RX on custom interface done
                usb_native_rx_stream.did_finish();
            } break;
        }
    }
}

// Called from CDC_Receive_FS callback function, this allows the communication
// thread to handle the incoming data
void usb_rx_process_packet(uint8_t *buf, uint32_t len, uint8_t endpoint_pair) {
    if (endpoint_pair == CDC_OUT_EP && usb_cdc_rx_stream.rx_end_) {
        usb_cdc_rx_stream.rx_end_ += len;
        osMessagePut(usb_event_queue, 5, 0);
    } else if (endpoint_pair == ODRIVE_OUT_EP && usb_native_rx_stream.rx_end_) {
        usb_native_rx_stream.rx_end_ += len;
        osMessagePut(usb_event_queue, 6, 0);
    }
}

void start_usb_server() {
    // Start USB communication thread
    osThreadDef(usb_server_thread_def, usb_server_thread, osPriorityNormal, 0, stack_size_usb_thread / sizeof(StackType_t));
    usb_thread = osThreadCreate(osThread(usb_server_thread_def), NULL);
}
