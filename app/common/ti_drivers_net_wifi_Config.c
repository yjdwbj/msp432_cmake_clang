

#include "Board.h"


/*
 *  =============================== WiFi ===============================
 *
 * This is the configuration structure for the WiFi module that will be used
 * as part of the SimpleLink SDK WiFi plugin. These are configured for SPI mode.
 * Any changes here will need to be configured on the CC31xx device as well
 */
//#include <ti/drivers/net/wifi/porting/SIMPLELINKWIFI.h>
//const SIMPLELINKWIFI_HWAttrsV1 wifiSimplelinkHWAttrs =
//{
//    .spiIndex = MSP_EXP432E401Y_SPI3,
//    .hostIRQPin = MSP_EXP432E401Y_HOST_IRQ,
//    .nHIBPin = MSP_EXP432E401Y_nHIB_pin,
//    .csPin = MSP_EXP432E401Y_CS_pin,
//    .maxDMASize = 1024,
//    .spiBitRate = 3000000
//};
//
//const uint_least8_t WiFi_count = 1;
//
//const WiFi_Config WiFi_config[1] =
//{
//    {
//        .hwAttrs = &wifiSimplelinkHWAttrs,
//    }
//};

#include <ti/drivers/net/wifi/simplelink.h>

/*
 *  =============================== SimpleLink Wifi ===============================
 */

/*
 *  ======== SimpleLinkWifi_config ========
 */
const SlWifiCC32XXConfig_t SimpleLinkWifiCC32XX_config = {
    .Mode = ROLE_STA,
//    .Ipv4Mode =  SL_NETCFG_IPV4_STA_ADDR_MODE ,
    .Ipv4Mode = SL_NETCFG_IPV6_ADDR_GLOBAL,
    .ConnectionPolicy = SL_WLAN_CONNECTION_POLICY(1,0,0,0),
    .PMPolicy = SL_WLAN_NORMAL_POLICY,
    .MaxSleepTimeMS = 0,
    .ScanPolicy = SL_WLAN_SCAN_POLICY(0,0),
    .ScanIntervalInSeconds = 0,
    .Ipv4Config = SL_NETCFG_ADDR_DHCP,
    .Ipv4 = 0,
    .IpMask = 0,
    .IpGateway = 0,
    .IpDnsServer = 0,
    .ProvisioningStop = 1,
    .DeleteAllProfile = 0
};
