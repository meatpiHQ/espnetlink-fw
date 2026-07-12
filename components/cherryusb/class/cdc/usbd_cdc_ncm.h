/*
 * usbd_cdc_ncm.h — CDC-NCM (NTB16) DEVICE class for CherryUSB.
 *
 * WiCAN-AUTHORED (2026-07-07, see PROVENANCE.md): CherryUSB has no
 * device-side NCM class (upstream ships usbh_cdc_ncm only). Written in the
 * style of usbd_cdc_ecm.c / usbd_rndis.c so it drops into the same
 * composite-device flow. NCM is the class Windows 10/11 binds natively
 * (UsbNcm.sys); the legacy RNDIS driver was removed in Windows 11 24H2.
 *
 * Scope: NTB16 only, one NDP per NTB, 16-bit format is all Windows/Linux/
 * macOS need at full speed. TX helper packs ONE datagram per NTB (simple,
 * latency-friendly at FS rates); RX parses any spec-conformant NTB the
 * host sends (multiple datagrams, chained NDPs).
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef USBD_CDC_NCM_H
#define USBD_CDC_NCM_H

#include "usb_cdc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Descriptor template. Differs from usb_cdc.h's CDC_NCM_DESCRIPTOR_INIT in
 * the two places Windows' UsbNcm is strict about (NCM 1.0 spec §5.3/§7.2):
 *  - the data interface has alt 0 with NO endpoints plus alt 1 with the
 *    two bulk endpoints (the host selects alt 1 to open the data path);
 *  - the data interface protocol is 0x01 ("Network Transfer Block").
 * bmNetworkCapabilities = 0x01: only SetEthernetPacketFilter on top of the
 * always-mandatory Get/SetNtbInputSize (4-byte form) + GetNtbParameters.
 * mac_str_idx names the string descriptor holding 12 uppercase hex chars —
 * the MAC the HOST's network interface uses (not the device's). */
#define CDC_NCM_ALT_DESCRIPTOR_LEN (8 + 9 + 5 + 5 + 13 + 6 + 7 + 9 + 9 + 7 + 7)
// clang-format off
#define CDC_NCM_ALT_DESCRIPTOR_INIT(bFirstInterface, int_ep, out_ep, in_ep, \
                                    wMaxPacketSize, wMaxSegmentSize, mac_str_idx) \
    /* Interface Association */                                                    \
    0x08,                                       /* bLength */                      \
    USB_DESCRIPTOR_TYPE_INTERFACE_ASSOCIATION,  /* bDescriptorType */              \
    bFirstInterface,                            /* bFirstInterface */              \
    0x02,                                       /* bInterfaceCount */              \
    USB_DEVICE_CLASS_CDC,                       /* bFunctionClass */               \
    CDC_NETWORK_CONTROL_MODEL,                  /* bFunctionSubClass */            \
    CDC_COMMON_PROTOCOL_NONE,                   /* bFunctionProtocol */            \
    0x00,                                       /* iFunction */                    \
    /* Communication interface */                                                  \
    0x09,                                       /* bLength */                      \
    USB_DESCRIPTOR_TYPE_INTERFACE,              /* bDescriptorType */              \
    bFirstInterface,                            /* bInterfaceNumber */             \
    0x00,                                       /* bAlternateSetting */            \
    0x01,                                       /* bNumEndpoints */                \
    USB_DEVICE_CLASS_CDC,                       /* bInterfaceClass */              \
    CDC_NETWORK_CONTROL_MODEL,                  /* bInterfaceSubClass */           \
    CDC_COMMON_PROTOCOL_NONE,                   /* bInterfaceProtocol */           \
    0x00,                                       /* iInterface */                   \
    0x05,                                       /* Header FD: bFunctionLength */   \
    CDC_CS_INTERFACE,                                                              \
    CDC_FUNC_DESC_HEADER,                                                          \
    WBVAL(CDC_V1_10),                           /* bcdCDC */                       \
    0x05,                                       /* Union FD: bFunctionLength */    \
    CDC_CS_INTERFACE,                                                              \
    CDC_FUNC_DESC_UNION,                                                           \
    bFirstInterface,                            /* bControlInterface */            \
    (uint8_t)(bFirstInterface + 1),             /* bSubordinateInterface0 */       \
    0x0D,                                       /* Ethernet FD: bFunctionLength */ \
    CDC_CS_INTERFACE,                                                              \
    CDC_FUNC_DESC_ETHERNET_NETWORKING,                                             \
    mac_str_idx,                                /* iMACAddress */                  \
    0x00, 0x00, 0x00, 0x00,                     /* bmEthernetStatistics */         \
    WBVAL(wMaxSegmentSize),                     /* wMaxSegmentSize */              \
    0x00, 0x00,                                 /* wNumberMCFilters */             \
    0x00,                                       /* bNumberPowerFilters */          \
    0x06,                                       /* NCM FD: bFunctionLength */      \
    CDC_CS_INTERFACE,                                                              \
    CDC_FUNC_DESC_NCM,                                                             \
    WBVAL(0x0100),                              /* bcdNcmVersion 1.0 */            \
    0x01,                                       /* bmNetworkCapabilities */        \
    0x07,                                       /* notify EP: bLength */           \
    USB_DESCRIPTOR_TYPE_ENDPOINT,                                                  \
    int_ep,                                     /* bEndpointAddress */             \
    0x03,                                       /* bmAttributes: interrupt */      \
    0x10, 0x00,                                 /* wMaxPacketSize 16 */            \
    0x10,                                       /* bInterval */                    \
    /* Data interface, alt 0 — no endpoints (mandatory, NCM 1.0 §5.3) */           \
    0x09,                                       /* bLength */                      \
    USB_DESCRIPTOR_TYPE_INTERFACE,                                                 \
    (uint8_t)(bFirstInterface + 1),             /* bInterfaceNumber */             \
    0x00,                                       /* bAlternateSetting */            \
    0x00,                                       /* bNumEndpoints */                \
    CDC_DATA_INTERFACE_CLASS,                   /* bInterfaceClass */              \
    0x00,                                       /* bInterfaceSubClass */           \
    0x01,                                       /* bInterfaceProtocol: NTB */      \
    0x00,                                       /* iInterface */                   \
    /* Data interface, alt 1 — the live data path */                               \
    0x09,                                       /* bLength */                      \
    USB_DESCRIPTOR_TYPE_INTERFACE,                                                 \
    (uint8_t)(bFirstInterface + 1),             /* bInterfaceNumber */             \
    0x01,                                       /* bAlternateSetting */            \
    0x02,                                       /* bNumEndpoints */                \
    CDC_DATA_INTERFACE_CLASS,                   /* bInterfaceClass */              \
    0x00,                                       /* bInterfaceSubClass */           \
    0x01,                                       /* bInterfaceProtocol: NTB */      \
    0x00,                                       /* iInterface */                   \
    0x07,                                       /* bulk OUT: bLength */            \
    USB_DESCRIPTOR_TYPE_ENDPOINT,                                                  \
    out_ep,                                     /* bEndpointAddress */             \
    0x02,                                       /* bmAttributes */                 \
    WBVAL(wMaxPacketSize),                                                         \
    0x00,                                       /* bInterval */                    \
    0x07,                                       /* bulk IN: bLength */             \
    USB_DESCRIPTOR_TYPE_ENDPOINT,                                                  \
    in_ep,                                      /* bEndpointAddress */             \
    0x02,                                       /* bmAttributes */                 \
    WBVAL(wMaxPacketSize),                                                         \
    0x00                                        /* bInterval */
// clang-format on

/* Largest NTB either direction; also what GET_NTB_PARAMETERS advertises.
 * Big enough for NTH16 + NDP16 + one full 1514-byte frame with alignment
 * slack, small enough to keep the DMA buffers cheap. */
#ifndef CONFIG_USBDEV_CDC_NCM_MAX_NTB_SIZE
#define CONFIG_USBDEV_CDC_NCM_MAX_NTB_SIZE 2048U
#endif

/* Init the COMM interface (registers all three endpoints + class requests).
 * Then register the DATA interface with usbd_cdc_ncm_init_data_intf — its
 * notify hook is how alt-setting changes (the host opening/closing the data
 * path) reach the class. */
struct usbd_interface *usbd_cdc_ncm_init_intf(struct usbd_interface *intf,
                                              const uint8_t int_ep,
                                              const uint8_t out_ep,
                                              const uint8_t in_ep);
struct usbd_interface *usbd_cdc_ncm_init_data_intf(struct usbd_interface *intf);

/* true once the host selected data-interface alt 1 (data path open) */
bool usbd_cdc_ncm_data_ready(void);

/* Arm the bulk OUT endpoint for one NTB / write one raw NTB.
 * Buffers must be USB-DMA-capable. */
int usbd_cdc_ncm_start_read(uint8_t *buf, uint32_t len);
int usbd_cdc_ncm_start_write(uint8_t *buf, uint32_t len);

/* Wrap ONE Ethernet frame in an NTB16 inside ntb_buf (cap = its size) and
 * start the bulk IN transfer. Returns the NTB length sent, or -USB_ERR_*. */
int usbd_cdc_ncm_eth_start_write(uint8_t *ntb_buf, uint32_t cap,
                                 const uint8_t *frame, uint32_t frame_len);

/* Walk a received NTB16, invoking cb per datagram (task context — call it
 * from wherever you handle usbd_cdc_ncm_data_recv_done, not the ISR).
 * Returns the datagram count, or -1 if the NTB is malformed. */
typedef void (*usbd_cdc_ncm_datagram_cb_t)(const uint8_t *frame,
                                           uint16_t len, void *arg);
int usbd_cdc_ncm_parse_ntb(const uint8_t *ntb, uint32_t len,
                           usbd_cdc_ncm_datagram_cb_t cb, void *arg);

/* Weak hooks, called from USB interrupt context on ESP ports:
 *  data_recv_done — one NTB landed in the start_read buffer (len bytes)
 *  data_send_done — the last start_write completed
 *  link_event     — host opened (true) / closed (false) the data path */
void usbd_cdc_ncm_data_recv_done(uint32_t len);
void usbd_cdc_ncm_data_send_done(uint32_t len);
void usbd_cdc_ncm_link_event(bool up);

#ifdef __cplusplus
}
#endif

#endif /* USBD_CDC_NCM_H */
