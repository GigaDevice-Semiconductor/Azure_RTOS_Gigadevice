/**************************************************************************/
/*                                                                        */
/*       Copyright (c) Microsoft Corporation. All rights reserved.        */
/*                                                                        */
/*       This software is licensed under the Microsoft Software License   */
/*       Terms for Microsoft Azure RTOS. Full text of the license can be  */
/*       found in the LICENSE file at https://aka.ms/AzureRTOS_EULA       */
/*       and in the root directory of this software.                      */
/*                                                                        */
/**************************************************************************/


/**************************************************************************/
/**************************************************************************/
/**                                                                       */
/** NetX Component                                                        */
/**                                                                       */
/**   Ethernet device driver for GD family microprocessors              */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

/* Indicate that driver source is being compiled.  */

#define NX_DRIVER_SOURCE


/****** DRIVER SPECIFIC ****** Start of part/vendor specific include area.  Include driver-specific include file here!  */


/* Determine if the driver uses IP deferred processing or direct ISR processing.  */

#define NX_DRIVER_ENABLE_DEFERRED                /* Define this to enable deferred ISR processing.  */

/* Determine if the packet transmit queue logic is required for this driver.   */

/* No, not required for this driver.  #define NX_DIRVER_INTERNAL_TRANSMIT_QUEUE   */

/* Include driver specific include file.  */
#include "nx_driver_gd32f450zk.h"
#include "gd32f4xx_enet_eval.h"


#define nx_driver_gd_ethernet_isr ENET_IRQHandler


/****** DRIVER SPECIFIC ****** End of part/vendor specific include file area!  */


/* Define the driver information structure that is only available within this file.  */

static NX_DRIVER_INFORMATION   nx_driver_information;


/****** DRIVER SPECIFIC ****** Start of part/vendor specific data area.  Include hardware-specific data here!  */

/* Define driver specific ethernet hardware address.  */

#ifndef NX_DRIVER_ETHERNET_MAC
UCHAR   _nx_driver_hardware_address[] = {0x2, 0xA, 0xF, 0xE, 0xD, 0x6};
#else
UCHAR   _nx_driver_hardware_address[] = NX_DRIVER_ETHERNET_MAC;  
#endif


/****** DRIVER SPECIFIC ****** End of part/vendor specific data area!  */


/* Define the routines for processing each driver entry request.  The contents of these routines will change with
   each driver. However, the main driver entry function will not change, except for the entry function name.  */
   
static VOID         _nx_driver_interface_attach(NX_IP_DRIVER *driver_req_ptr);
static VOID         _nx_driver_initialize(NX_IP_DRIVER *driver_req_ptr);
static VOID         _nx_driver_enable(NX_IP_DRIVER *driver_req_ptr);
static VOID         _nx_driver_disable(NX_IP_DRIVER *driver_req_ptr);
static VOID         _nx_driver_packet_send(NX_IP_DRIVER *driver_req_ptr);
static VOID         _nx_driver_multicast_join(NX_IP_DRIVER *driver_req_ptr);
static VOID         _nx_driver_multicast_leave(NX_IP_DRIVER *driver_req_ptr);
static VOID         _nx_driver_get_status(NX_IP_DRIVER *driver_req_ptr);
#ifdef NX_DRIVER_ENABLE_DEFERRED
static VOID         _nx_driver_deferred_processing(NX_IP_DRIVER *driver_req_ptr);
#endif
static VOID         _nx_driver_transfer_to_netx(NX_IP *ip_ptr, NX_PACKET *packet_ptr);
#ifdef NX_DIRVER_INTERNAL_TRANSMIT_QUEUE
static VOID         _nx_driver_transmit_packet_enqueue(NX_PACKET *packet_ptr)
static NX_PACKET    *_nx_driver_transmit_packet_dequeue(VOID)
#endif
#ifdef NX_ENABLE_INTERFACE_CAPABILITY
static VOID         _nx_driver_capability_get(NX_IP_DRIVER *driver_req_ptr);
static VOID         _nx_driver_capability_set(NX_IP_DRIVER *driver_req_ptr);
#endif /* NX_ENABLE_INTERFACE_CAPABILITY */

/* Define the prototypes for the hardware implementation of this driver. The contents of these routines are
   driver-specific.  */

static UINT         _nx_driver_hardware_initialize(NX_IP_DRIVER *driver_req_ptr); 
static UINT         _nx_driver_hardware_enable(NX_IP_DRIVER *driver_req_ptr); 
static UINT         _nx_driver_hardware_disable(NX_IP_DRIVER *driver_req_ptr); 
static UINT         _nx_driver_hardware_packet_send(NX_PACKET *packet_ptr); 
static UINT         _nx_driver_hardware_multicast_join(NX_IP_DRIVER *driver_req_ptr);
static UINT         _nx_driver_hardware_multicast_leave(NX_IP_DRIVER *driver_req_ptr);
static UINT         _nx_driver_hardware_get_status(NX_IP_DRIVER *driver_req_ptr);
static VOID         _nx_driver_hardware_packet_transmitted(VOID);
static VOID         _nx_driver_hardware_packet_received(VOID);
#ifdef NX_ENABLE_INTERFACE_CAPABILITY
static UINT         _nx_driver_hardware_capability_set(NX_IP_DRIVER *driver_req_ptr);
#endif 

/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    nx_driver_gd                                       PORTABLE C      */
/*                                                           5.0          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    Yuxin Zhou, Microsoft Corporation                                   */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This is the entry point of the NetX Ethernet Driver. This driver    */ 
/*    function is responsible for initializing the Ethernet controller,   */ 
/*    enabling or disabling the controller as need, preparing             */ 
/*    a packet for transmission, and getting status information.          */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    driver_req_ptr                        The driver request from the   */ 
/*                                            IP layer.                   */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _nx_driver_interface_attach           Process attach request        */ 
/*    _nx_driver_initialize                 Process initialize request    */ 
/*    _nx_driver_enable                     Process link enable request   */ 
/*    _nx_driver_disable                    Process link disable request  */ 
/*    _nx_driver_packet_send                Process send packet requests  */ 
/*    _nx_driver_multicast_join             Process multicast join request*/ 
/*    _nx_driver_multicast_leave            Process multicast leave req   */ 
/*    _nx_driver_get_status                 Process get status request    */ 
/*    _nx_driver_deferred_processing        Drive deferred processing     */ 
/*                                                                        */
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    IP layer                                                            */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  02-01-2018     Yuxin Zhou               Initial Version 5.0           */ 
/*                                                                        */ 
/**************************************************************************/ 
/****** DRIVER SPECIFIC ****** Start of part/vendor specific global driver entry function name.  */
VOID  nx_driver_gd(NX_IP_DRIVER *driver_req_ptr)
/****** DRIVER SPECIFIC ****** End of part/vendor specific global driver entry function name.  */
{
    
    /* Default to successful return.  */
    driver_req_ptr -> nx_ip_driver_status =  NX_SUCCESS;
    
    /* Process according to the driver request type in the IP control 
       block.  */
    switch (driver_req_ptr -> nx_ip_driver_command)
    {
        
    case NX_LINK_INTERFACE_ATTACH:
       
        /* Process link interface attach requests.  */
        _nx_driver_interface_attach(driver_req_ptr);
        break;
        
    case NX_LINK_INITIALIZE:
    {

        /* Process link initialize requests.  */
        _nx_driver_initialize(driver_req_ptr);
        break;
    }
        
    case NX_LINK_ENABLE:
    {
    
        /* Process link enable requests.  */
        _nx_driver_enable(driver_req_ptr);
        break;
    }
        
    case NX_LINK_DISABLE:
    {
    
        /* Process link disable requests.  */
        _nx_driver_disable(driver_req_ptr);
        break;
    } 
        
        
    case NX_LINK_ARP_SEND:
    case NX_LINK_ARP_RESPONSE_SEND:
    case NX_LINK_PACKET_BROADCAST:
    case NX_LINK_RARP_SEND:
    case NX_LINK_PACKET_SEND:
    {
            
        /* Process packet send requests.  */
        _nx_driver_packet_send(driver_req_ptr);
        break;
    }
        
        
    case NX_LINK_MULTICAST_JOIN:
    {
            
        /* Process multicast join requests.  */
        _nx_driver_multicast_join(driver_req_ptr);
        break;
    }
    
        
    case NX_LINK_MULTICAST_LEAVE:
    {
            
        /* Process multicast leave requests.  */
        _nx_driver_multicast_leave(driver_req_ptr);
        break;
    }
        
    case NX_LINK_GET_STATUS:
    {
            
        /* Process get status requests.  */
        _nx_driver_get_status(driver_req_ptr);
        break;
    }
#ifdef NX_DRIVER_ENABLE_DEFERRED        
    case NX_LINK_DEFERRED_PROCESSING:
    { 
        
        /* Process driver deferred requests.  */
        
        /* Process a device driver function on behave of the IP thread. */
        _nx_driver_deferred_processing(driver_req_ptr);
        break;
    }
#endif
#ifdef NX_ENABLE_INTERFACE_CAPABILITY
    case NX_INTERFACE_CAPABILITY_GET:
    {

        /* Process get capability requests.  */
        _nx_driver_capability_get(driver_req_ptr);
        break;
    }
    
    case NX_INTERFACE_CAPABILITY_SET:
    {

        /* Process set capability requests.  */
        _nx_driver_capability_set(driver_req_ptr);
        break;
    }
#endif /* NX_ENABLE_INTERFACE_CAPABILITY */    
    default:
           
        /* Invalid driver request.  */
        
        /* Return the unhandled command status.  */
        driver_req_ptr -> nx_ip_driver_status =  NX_UNHANDLED_COMMAND;

        /* Default to successful return.  */
        driver_req_ptr -> nx_ip_driver_status =  NX_DRIVER_ERROR;
    }
}


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _nx_driver_interface_attach                         PORTABLE C      */ 
/*                                                           5.0          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    Yuxin Zhou, Microsoft Corporation                                   */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This function processing the interface attach request.              */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    driver_req_ptr                        Driver command from the IP    */ 
/*                                            thread                      */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Driver entry function                                               */
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  02-01-2018     Yuxin Zhou               Initial Version 5.0           */ 
/*                                                                        */ 
/**************************************************************************/ 
static VOID  _nx_driver_interface_attach(NX_IP_DRIVER *driver_req_ptr)
{

    
    /* Setup the driver's interface.  This example is for a simple one-interface
       Ethernet driver. Additional logic is necessary for multiple port devices.  */
    nx_driver_information.nx_driver_information_interface =  driver_req_ptr -> nx_ip_driver_interface;
    
#ifdef NX_ENABLE_INTERFACE_CAPABILITY
    driver_req_ptr -> nx_ip_driver_interface -> nx_interface_capability_flag = NX_DRIVER_CAPABILITY;
#endif /* NX_ENABLE_INTERFACE_CAPABILITY */
    
    /* Return successful status.  */
    driver_req_ptr -> nx_ip_driver_status =  NX_SUCCESS;
}


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _nx_driver_initialize                               PORTABLE C      */ 
/*                                                           5.0          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    Yuxin Zhou, Microsoft Corporation                                   */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This function processing the initialize request.  The processing    */
/*    in this function is generic. All ethernet controller logic is to    */ 
/*    be placed in _nx_driver_hardware_initialize.                        */  
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    driver_req_ptr                        Driver command from the IP    */ 
/*                                            thread                      */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _nx_driver_hardware_initialize        Process initialize request    */ 
/*                                                                        */
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Driver entry function                                               */
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  02-01-2018     Yuxin Zhou               Initial Version 5.0           */ 
/*                                                                        */ 
/**************************************************************************/ 
static VOID  _nx_driver_initialize(NX_IP_DRIVER *driver_req_ptr)
{

NX_IP           *ip_ptr;
NX_INTERFACE    *interface_ptr;
UINT            status;
        
        
    /* Setup the IP pointer from the driver request.  */
    ip_ptr =  driver_req_ptr -> nx_ip_driver_ptr;
    
    /* Setup interface pointer.  */
    interface_ptr = driver_req_ptr -> nx_ip_driver_interface;

    /* Initialize the driver's information structure.  */

    /* Default IP pointer to NULL.  */
    nx_driver_information.nx_driver_information_ip_ptr =               NX_NULL;

    /* Setup the driver state to not initialized.  */
    nx_driver_information.nx_driver_information_state =                NX_DRIVER_STATE_NOT_INITIALIZED;

    /* Setup the default packet pool for the driver's received packets.  */
    nx_driver_information.nx_driver_information_packet_pool_ptr = ip_ptr -> nx_ip_default_packet_pool;

    /* Clear the deferred events for the driver.  */
    nx_driver_information.nx_driver_information_deferred_events =       0;

#ifdef NX_DIRVER_INTERNAL_TRANSMIT_QUEUE

    /* Clear the transmit queue count and head pointer.  */
    nx_driver_information.nx_driver_transmit_packets_queued =  0;
    nx_driver_information.nx_driver_transmit_queue_head =      NX_NULL;
    nx_driver_information.nx_driver_transmit_queue_tail =      NX_NULL;
#endif

    /* Call the hardware-specific ethernet controller initialization.  */
    status =  _nx_driver_hardware_initialize(driver_req_ptr);

    /* Determine if the request was successful.  */
    if (status == NX_SUCCESS)
    {

        /* Successful hardware initialization.  */

        /* Setup driver information to point to IP pointer.  */
        nx_driver_information.nx_driver_information_ip_ptr = driver_req_ptr -> nx_ip_driver_ptr;
            
        /* Setup the link maximum transfer unit. */
        interface_ptr -> nx_interface_ip_mtu_size =  NX_DRIVER_ETHERNET_MTU - NX_DRIVER_ETHERNET_FRAME_SIZE;
            
        /* Setup the physical address of this IP instance.  Increment the 
           physical address lsw to simulate multiple nodes hanging on the
           ethernet.  */
        interface_ptr -> nx_interface_physical_address_msw =  
                (ULONG)((_nx_driver_hardware_address[0] << 8) | (_nx_driver_hardware_address[1]));
        interface_ptr -> nx_interface_physical_address_lsw =  
                (ULONG)((_nx_driver_hardware_address[2] << 24) | (_nx_driver_hardware_address[3] << 16) | 
                        (_nx_driver_hardware_address[4] << 8) | (_nx_driver_hardware_address[5]));
                
        /* Indicate to the IP software that IP to physical mapping
           is required.  */
        interface_ptr -> nx_interface_address_mapping_needed =  NX_TRUE;
            
        /* Move the driver's state to initialized.  */
        nx_driver_information.nx_driver_information_state = NX_DRIVER_STATE_INITIALIZED;

        /* Indicate successful initialize.  */
        driver_req_ptr -> nx_ip_driver_status =  NX_SUCCESS;
    }
    else
    {
        
        /* Initialization failed.  Indicate that the request failed.  */
        driver_req_ptr -> nx_ip_driver_status =   NX_DRIVER_ERROR;
    }
}


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _nx_driver_enable                                   PORTABLE C      */ 
/*                                                           5.0          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    Yuxin Zhou, Microsoft Corporation                                   */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This function processing the initialize request. The processing     */
/*    in this function is generic. All ethernet controller logic is to    */ 
/*    be placed in _nx_driver_hardware_enable.                            */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    driver_req_ptr                        Driver command from the IP    */ 
/*                                            thread                      */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _nx_driver_hardware_enable            Process enable request        */ 
/*                                                                        */
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Driver entry function                                               */
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  02-01-2018     Yuxin Zhou               Initial Version 5.0           */ 
/*                                                                        */ 
/**************************************************************************/ 
static VOID  _nx_driver_enable(NX_IP_DRIVER *driver_req_ptr)
{

UINT            status;
    
    /* See if we can honor the NX_LINK_ENABLE request.  */
    if (nx_driver_information.nx_driver_information_state < NX_DRIVER_STATE_INITIALIZED)
    {

        /* Mark the request as not successful.  */
        driver_req_ptr -> nx_ip_driver_status =  NX_DRIVER_ERROR;                      
        return;
    }                
        
    /* Check if it is enabled by someone already */
    if (nx_driver_information.nx_driver_information_state >=  NX_DRIVER_STATE_LINK_ENABLED)
    {

        /* Yes, the request has already been made.  */
        driver_req_ptr -> nx_ip_driver_status =  NX_ALREADY_ENABLED;
        return;
    }

    /* Call hardware specific enable.  */
    status =  _nx_driver_hardware_enable(driver_req_ptr);

    /* Was the hardware enable successful?  */
    if (status == NX_SUCCESS)
    {

        /* Update the driver state to link enabled.  */
        nx_driver_information.nx_driver_information_state = NX_DRIVER_STATE_LINK_ENABLED;

        /* Mark request as successful.  */
        driver_req_ptr -> nx_ip_driver_status =  NX_SUCCESS;
        
        /* Mark the IP instance as link up.  */        
        driver_req_ptr -> nx_ip_driver_interface -> nx_interface_link_up =  NX_TRUE;
    }
    else
    {
        
        /* Enable failed.  Indicate that the request failed.  */
        driver_req_ptr -> nx_ip_driver_status =   NX_DRIVER_ERROR;
    }
}


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _nx_driver_disable                                  PORTABLE C      */ 
/*                                                           5.0          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    Yuxin Zhou, Microsoft Corporation                                   */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This function processing the disable request. The processing        */
/*    in this function is generic. All ethernet controller logic is to    */ 
/*    be placed in _nx_driver_hardware_disable.                           */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    driver_req_ptr                        Driver command from the IP    */ 
/*                                            thread                      */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _nx_driver_hardware_disable           Process disable request       */ 
/*                                                                        */
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Driver entry function                                               */
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  02-01-2018     Yuxin Zhou               Initial Version 5.0           */ 
/*                                                                        */ 
/**************************************************************************/ 
static VOID  _nx_driver_disable(NX_IP_DRIVER *driver_req_ptr)
{

NX_IP           *ip_ptr;
UINT            status;


    /* Setup the IP pointer from the driver request.  */
    ip_ptr =  driver_req_ptr -> nx_ip_driver_ptr;

    /* Check if the link is enabled.  */
    if (nx_driver_information.nx_driver_information_state !=  NX_DRIVER_STATE_LINK_ENABLED)
    {

        /* The link is not enabled, so just return an error.  */
        driver_req_ptr -> nx_ip_driver_status =  NX_DRIVER_ERROR;
        return;
    }

    /* Call hardware specific disable.  */
    status =  _nx_driver_hardware_disable(driver_req_ptr);

    /* Was the hardware disable successful?  */
    if (status == NX_SUCCESS)
    {
    
        /* Mark the IP instance as link down.  */        
        ip_ptr -> nx_ip_driver_link_up =  NX_FALSE;

        /* Update the driver state back to initialized.  */
        nx_driver_information.nx_driver_information_state =  NX_DRIVER_STATE_INITIALIZED;
        
        /* Mark request as successful.  */
        driver_req_ptr -> nx_ip_driver_status =  NX_SUCCESS;
    }
    else
    {

        /* Disable failed, return an error.  */    
        driver_req_ptr -> nx_ip_driver_status =  NX_DRIVER_ERROR;
    }
}


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _nx_driver_packet_send                              PORTABLE C      */ 
/*                                                           5.0          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    Yuxin Zhou, Microsoft Corporation                                   */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This function processing the packet send request. The processing    */
/*    in this function is generic. All ethernet controller packet send    */ 
/*    logic is to be placed in _nx_driver_hardware_packet_send.           */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    driver_req_ptr                        Driver command from the IP    */ 
/*                                            thread                      */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _nx_driver_hardware_packet_send       Process packet send request   */ 
/*                                                                        */
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Driver entry function                                               */
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  02-01-2018     Yuxin Zhou               Initial Version 5.0           */ 
/*                                                                        */ 
/**************************************************************************/ 
static VOID  _nx_driver_packet_send(NX_IP_DRIVER *driver_req_ptr)
{

NX_PACKET       *packet_ptr;
ULONG           *ethernet_frame_ptr;
UINT            status;

    
    /* Check to make sure the link is up.  */
    if (nx_driver_information.nx_driver_information_state != NX_DRIVER_STATE_LINK_ENABLED)
    {

        /* Inidate an unsuccessful packet send.  */
        driver_req_ptr -> nx_ip_driver_status =  NX_DRIVER_ERROR;

        /* Link is not up, simply free the packet.  */
        nx_packet_transmit_release(driver_req_ptr -> nx_ip_driver_packet);
        return;
    }                
    
    /* Process driver send packet.  */
    
    /* Place the ethernet frame at the front of the packet.  */
    packet_ptr =  driver_req_ptr -> nx_ip_driver_packet;
        
    /* Adjust the prepend pointer.  */
    packet_ptr -> nx_packet_prepend_ptr =  
            packet_ptr -> nx_packet_prepend_ptr - NX_DRIVER_ETHERNET_FRAME_SIZE;
        
    /* Adjust the packet length.  */
    packet_ptr -> nx_packet_length = packet_ptr -> nx_packet_length + NX_DRIVER_ETHERNET_FRAME_SIZE;
            
    /* Setup the ethernet frame pointer to build the ethernet frame.  Backup another 2
      * bytes to get 32-bit word alignment.  */
    ethernet_frame_ptr =  (ULONG *) (packet_ptr -> nx_packet_prepend_ptr - 2);
        
    /* Set up the hardware addresses in the Ethernet header. */
    *ethernet_frame_ptr       =  driver_req_ptr -> nx_ip_driver_physical_address_msw;
    *(ethernet_frame_ptr + 1) =  driver_req_ptr -> nx_ip_driver_physical_address_lsw;
        
    *(ethernet_frame_ptr + 2) =  (driver_req_ptr -> nx_ip_driver_interface -> nx_interface_physical_address_msw << 16) |
        (driver_req_ptr -> nx_ip_driver_interface -> nx_interface_physical_address_lsw >> 16);
    *(ethernet_frame_ptr + 3) =  (driver_req_ptr -> nx_ip_driver_interface -> nx_interface_physical_address_lsw << 16);

    /* Set up the frame type field in the Ethernet harder. */
    if ((driver_req_ptr -> nx_ip_driver_command == NX_LINK_ARP_SEND)||
        (driver_req_ptr -> nx_ip_driver_command == NX_LINK_ARP_RESPONSE_SEND))
    {

        *(ethernet_frame_ptr + 3) |= NX_DRIVER_ETHERNET_ARP;
    }
    else if(driver_req_ptr -> nx_ip_driver_command == NX_LINK_RARP_SEND)
    {

        *(ethernet_frame_ptr + 3) |= NX_DRIVER_ETHERNET_RARP;
    }

#ifdef FEATURE_NX_IPV6
    else if(packet_ptr -> nx_packet_ip_version == NX_IP_VERSION_V6)
    {

        *(ethernet_frame_ptr + 3) |= NX_DRIVER_ETHERNET_IPV6;
    }
#endif

    else
    {

        *(ethernet_frame_ptr + 3) |= NX_DRIVER_ETHERNET_IP;
    }

    /* Endian swapping if NX_LITTLE_ENDIAN is defined.  */
    NX_CHANGE_ULONG_ENDIAN(*(ethernet_frame_ptr));
    NX_CHANGE_ULONG_ENDIAN(*(ethernet_frame_ptr + 1));
    NX_CHANGE_ULONG_ENDIAN(*(ethernet_frame_ptr + 2));
    NX_CHANGE_ULONG_ENDIAN(*(ethernet_frame_ptr + 3));
        
    /* Determine if the packet exceeds the driver's MTU.  */
    if (packet_ptr -> nx_packet_length > NX_DRIVER_ETHERNET_MTU)
    {
    
        /* This packet exceeds the size of the driver's MTU. Simply throw it away! */

        /* Remove the Ethernet header.  */
        NX_DRIVER_ETHERNET_HEADER_REMOVE(packet_ptr);
        
        /* Indicate an unsuccessful packet send.  */
        driver_req_ptr -> nx_ip_driver_status =  NX_DRIVER_ERROR;

        /* Link is not up, simply free the packet.  */
        nx_packet_transmit_release(packet_ptr);
        return;
    }

    /* Transmit the packet through the Ethernet controller low level access routine. */
    status = _nx_driver_hardware_packet_send(packet_ptr);

    /* Determine if there was an error.  */
    if (status != NX_SUCCESS)
    {

        /* Driver's hardware send packet routine failed to send the packet.  */

        /* Remove the Ethernet header.  */
        NX_DRIVER_ETHERNET_HEADER_REMOVE(packet_ptr);
        
        /* Indicate an unsuccessful packet send.  */
        driver_req_ptr -> nx_ip_driver_status =  NX_DRIVER_ERROR;

        /* Link is not up, simply free the packet.  */
        nx_packet_transmit_release(packet_ptr);
    }
    else
    {

        /* Set the status of the request.  */    
        driver_req_ptr -> nx_ip_driver_status =  NX_SUCCESS;
    }
}


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _nx_driver_multicast_join                           PORTABLE C      */ 
/*                                                           5.0          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    Yuxin Zhou, Microsoft Corporation                                   */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This function processing the multicast join request. The processing */
/*    in this function is generic. All ethernet controller multicast join */ 
/*    logic is to be placed in _nx_driver_hardware_multicast_join.        */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    driver_req_ptr                        Driver command from the IP    */ 
/*                                            thread                      */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _nx_driver_hardware_multicast_join    Process multicast join request*/ 
/*                                                                        */
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Driver entry function                                               */
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  02-01-2018     Yuxin Zhou               Initial Version 5.0           */ 
/*                                                                        */ 
/**************************************************************************/ 
static VOID  _nx_driver_multicast_join(NX_IP_DRIVER *driver_req_ptr)
{

UINT        status;


    if (nx_driver_information.nx_driver_information_state >= NX_DRIVER_STATE_INITIALIZED)
    {
        
        /* Call hardware specific multicast join function. */
        status =  _nx_driver_hardware_multicast_join(driver_req_ptr);
    }
    else
    {
        
        status =  NX_NOT_ENABLED;
    }
    
    /* Determine if there was an error.  */
    if (status != NX_SUCCESS)
    {

        /* Indicate an unsuccessful request.  */
        driver_req_ptr -> nx_ip_driver_status =  NX_DRIVER_ERROR;
    }
    else
    {

        /* Indicate the request was successful.   */    
        driver_req_ptr -> nx_ip_driver_status =  NX_SUCCESS;
    }
}


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _nx_driver_multicast_leave                          PORTABLE C      */ 
/*                                                           5.0          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    Yuxin Zhou, Microsoft Corporation                                   */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This function processing the multicast leave request. The           */ 
/*    processing in this function is generic. All ethernet controller     */ 
/*    multicast leave logic is to be placed in                            */ 
/*    _nx_driver_hardware_multicast_leave.                                */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    driver_req_ptr                        Driver command from the IP    */ 
/*                                            thread                      */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _nx_driver_hardware_multicast_leave   Process multicast leave req   */ 
/*                                                                        */
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Driver entry function                                               */
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  02-01-2018     Yuxin Zhou               Initial Version 5.0           */ 
/*                                                                        */ 
/**************************************************************************/ 
static VOID  _nx_driver_multicast_leave(NX_IP_DRIVER *driver_req_ptr)
{

UINT        status;

    if (nx_driver_information.nx_driver_information_state >= NX_DRIVER_STATE_INITIALIZED)
    {
        
        /* Call hardware specific multicast leave function. */
        status =  _nx_driver_hardware_multicast_leave(driver_req_ptr);
    }
    else
    {
        
        status =  NX_NOT_ENABLED;
    }    
    
    /* Determine if there was an error.  */
    if (status != NX_SUCCESS)
    {

        /* Indicate an unsuccessful request.  */
        driver_req_ptr -> nx_ip_driver_status =  NX_DRIVER_ERROR;
    }
    else
    {

        /* Indicate the request was successful.   */    
        driver_req_ptr -> nx_ip_driver_status =  NX_SUCCESS;
    }
}


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _nx_driver_get_status                               PORTABLE C      */ 
/*                                                           5.0          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    Yuxin Zhou, Microsoft Corporation                                   */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This function processing the get status request. The processing     */
/*    in this function is generic. All ethernet controller get status     */ 
/*    logic is to be placed in _nx_driver_hardware_get_status.            */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    driver_req_ptr                        Driver command from the IP    */ 
/*                                            thread                      */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _nx_driver_hardware_get_status        Process get status request    */ 
/*                                                                        */
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Driver entry function                                               */
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  02-01-2018     Yuxin Zhou               Initial Version 5.0           */ 
/*                                                                        */ 
/**************************************************************************/ 
static VOID  _nx_driver_get_status(NX_IP_DRIVER *driver_req_ptr)
{

UINT        status;


    /* Call hardware specific get status function. */
    status =  _nx_driver_hardware_get_status(driver_req_ptr);
    
    /* Determine if there was an error.  */
    if (status != NX_SUCCESS)
    {

        /* Indicate an unsuccessful request.  */
        driver_req_ptr -> nx_ip_driver_status =  NX_DRIVER_ERROR;
    }
    else
    {

        /* Indicate the request was successful.   */    
        driver_req_ptr -> nx_ip_driver_status =  NX_SUCCESS;
    }
}



#ifdef NX_ENABLE_INTERFACE_CAPABILITY
/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _nx_driver_capability_get                         PORTABLE C        */ 
/*                                                           5.0          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    Yuxin Zhou, Microsoft Corporation                                   */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This function processing the get capability request.                */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    driver_req_ptr                        Driver command from the IP    */ 
/*                                            thread                      */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Driver entry function                                               */
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  01-01-2014     Yuxin Zhou               Initial Version 5.0           */ 
/*                                                                        */ 
/**************************************************************************/ 
static VOID  _nx_driver_capability_get(NX_IP_DRIVER *driver_req_ptr)
{
    
    /* Return the capability of the Ethernet controller.  */
    *(driver_req_ptr -> nx_ip_driver_return_ptr) = NX_DRIVER_CAPABILITY;
    
    /* Return the success status.  */
    driver_req_ptr -> nx_ip_driver_status =  NX_SUCCESS;
}


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _nx_driver_capability_set                         PORTABLE C        */ 
/*                                                           5.0          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    Yuxin Zhou, Microsoft Corporation                                   */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This function processing the set capability request.                */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    driver_req_ptr                        Driver command from the IP    */ 
/*                                            thread                      */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Driver entry function                                               */
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  01-01-2014     Yuxin Zhou               Initial Version 5.0           */ 
/*                                                                        */ 
/**************************************************************************/ 
static VOID  _nx_driver_capability_set(NX_IP_DRIVER *driver_req_ptr)
{

UINT        status;


    /* Call hardware specific get status function. */
    status =  _nx_driver_hardware_capability_set(driver_req_ptr);
    
    /* Determine if there was an error.  */
    if (status != NX_SUCCESS)
    {

        /* Indicate an unsuccessful request.  */
        driver_req_ptr -> nx_ip_driver_status =  NX_DRIVER_ERROR;
    }
    else
    {

        /* Indicate the request was successful.   */    
        driver_req_ptr -> nx_ip_driver_status =  NX_SUCCESS;
    }
}
#endif /* NX_ENABLE_INTERFACE_CAPABILITY */



#ifdef NX_DRIVER_ENABLE_DEFERRED
/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _nx_driver_deferred_processing                      PORTABLE C      */ 
/*                                                           5.0          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    XC, Microsoft Corporation                                           */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This function processing the deferred ISR action within the context */ 
/*    of the IP thread.                                                   */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    driver_req_ptr                        Driver command from the IP    */ 
/*                                            thread                      */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _nx_driver_packet_transmitted        Clean up after transmission    */
/*    _nx_driver_packet_received           Process a received packet      */
/*                                                                        */
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Driver entry function                                               */
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  02-01-2018     Yuxin Zhou               Initial Version 5.0           */ 
/*                                                                        */ 
/**************************************************************************/ 
static VOID  _nx_driver_deferred_processing(NX_IP_DRIVER *driver_req_ptr)
{

TX_INTERRUPT_SAVE_AREA

ULONG       deferred_events;

    
    /* Disable interrupts.  */
    TX_DISABLE

    /* Pickup deferred events.  */
    deferred_events =  nx_driver_information.nx_driver_information_deferred_events;
    nx_driver_information.nx_driver_information_deferred_events =  0;

    /* Restore interrupts.  */
    TX_RESTORE
        
    /* Check for a transmit complete event.  */
    if(deferred_events & NX_DRIVER_DEFERRED_PACKET_TRANSMITTED)
    {
        
        /* Process transmitted packet(s).  */
        _nx_driver_hardware_packet_transmitted();
    }    
    
    /* Check for received packet.  */
    if(deferred_events & NX_DRIVER_DEFERRED_PACKET_RECEIVED)
    {

        /* Process received packet(s).  */
        _nx_driver_hardware_packet_received();
    }

    /* Mark request as successful.  */    
    driver_req_ptr->nx_ip_driver_status =  NX_SUCCESS;
}
#endif


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _nx_driver_transfer_to_netx                         PORTABLE C      */ 
/*                                                           5.0          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    Yuxin Zhou, Microsoft Corporation                                   */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This function processing incoming packets.  This routine would      */
/*    be called from the driver-specific receive packet processing        */ 
/*    function _nx_driver_hardware_packet_received.                       */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    ip_ptr                                Pointer to IP protocol block  */ 
/*    packet_ptr                            Packet pointer                */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Error indication                                                    */
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _nx_ip_packet_receive                 NetX IP packet receive        */ 
/*    _nx_ip_packet_deferred_receive        NetX IP packet receive        */ 
/*    _nx_arp_packet_deferred_receive       NetX ARP packet receive       */ 
/*    _nx_rarp_packet_deferred_receive      NetX RARP packet receive      */ 
/*    _nx_packet_release                    Release packet                */ 
/*                                                                        */
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    _nx_driver_hardware_packet_received   Driver packet receive function*/ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  02-01-2018     Yuxin Zhou               Initial Version 5.0           */ 
/*                                                                        */ 
/**************************************************************************/ 
static VOID _nx_driver_transfer_to_netx(NX_IP *ip_ptr, NX_PACKET *packet_ptr)
{

USHORT    packet_type;


    packet_ptr -> nx_packet_ip_interface = nx_driver_information.nx_driver_information_interface;

    /* Pickup the packet header to determine where the packet needs to be
       sent.  */
    packet_type =  (USHORT)(((UINT) (*(packet_ptr -> nx_packet_prepend_ptr+12))) << 8) | 
        ((UINT) (*(packet_ptr -> nx_packet_prepend_ptr+13)));

    /* Route the incoming packet according to its ethernet type.  */
    if (packet_type == NX_DRIVER_ETHERNET_IP || packet_type == NX_DRIVER_ETHERNET_IPV6)
    {

        /* Note:  The length reported by some Ethernet hardware includes 
           bytes after the packet as well as the Ethernet header.  In some 
           cases, the actual packet length after the Ethernet header should 
           be derived from the length in the IP header (lower 16 bits of
           the first 32-bit word).  */

        /* Clean off the Ethernet header.  */
        packet_ptr -> nx_packet_prepend_ptr =  
            packet_ptr -> nx_packet_prepend_ptr + NX_DRIVER_ETHERNET_FRAME_SIZE;
 
        /* Adjust the packet length.  */
        packet_ptr -> nx_packet_length =  
            packet_ptr -> nx_packet_length - NX_DRIVER_ETHERNET_FRAME_SIZE;

        /* Route to the ip receive function.  */
#ifdef NX_DRIVER_ENABLE_DEFERRED
        _nx_ip_packet_deferred_receive(ip_ptr, packet_ptr);
#else
        _nx_ip_packet_receive(ip_ptr, packet_ptr);
#endif
    }
    else if (packet_type == NX_DRIVER_ETHERNET_ARP)
    {

        /* Clean off the Ethernet header.  */
        packet_ptr -> nx_packet_prepend_ptr =  
            packet_ptr -> nx_packet_prepend_ptr + NX_DRIVER_ETHERNET_FRAME_SIZE;

        /* Adjust the packet length.  */
        packet_ptr -> nx_packet_length =  
            packet_ptr -> nx_packet_length - NX_DRIVER_ETHERNET_FRAME_SIZE;

        /* Route to the ARP receive function.  */
        _nx_arp_packet_deferred_receive(ip_ptr, packet_ptr);
    }
    else if (packet_type == NX_DRIVER_ETHERNET_RARP)
    {

        /* Clean off the Ethernet header.  */
        packet_ptr -> nx_packet_prepend_ptr =  
            packet_ptr -> nx_packet_prepend_ptr + NX_DRIVER_ETHERNET_FRAME_SIZE;

        /* Adjust the packet length.  */
        packet_ptr -> nx_packet_length =  
            packet_ptr -> nx_packet_length - NX_DRIVER_ETHERNET_FRAME_SIZE;

        /* Route to the RARP receive function.  */
        _nx_rarp_packet_deferred_receive(ip_ptr, packet_ptr);
    }
    else
    {
        /* Invalid ethernet header... release the packet.  */
        nx_packet_release(packet_ptr);
    }
}            


#ifdef NX_DIRVER_INTERNAL_TRANSMIT_QUEUE
/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _nx_driver_transmit_packet_enqueue                  PORTABLE C      */ 
/*                                                           5.0          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    Yuxin Zhou, Microsoft Corporation                                   */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This function queues a transmit packet when the hardware transmit   */ 
/*    queue does not have the resources (buffer descriptors, etc.) to     */ 
/*    send the packet.  The queue is maintained as a singularly linked-   */ 
/*    list with head and tail pointers. The maximum number of packets on  */ 
/*    the transmit queue is regulated by the constant                     */ 
/*    NX_DRIVER_MAX_TRANSMIT_QUEUE_DEPTH. When this number is exceeded,   */  
/*    the oldest packet is discarded after the new packet is queued.      */ 
/*                                                                        */ 
/*    Note: that it is assumed further driver interrupts are locked out   */ 
/*    during the call to this driver utility.                             */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    packet_ptr                            Packet pointer                */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _nx_packet_transmit_release           Release packet                */ 
/*                                                                        */
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    _nx_driver_hardware_packet_send       Driver packet send function   */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  02-01-2018     Yuxin Zhou               Initial Version 5.0           */ 
/*                                                                        */ 
/**************************************************************************/ 
static VOID _nx_driver_transmit_packet_enqueue(NX_PACKET *packet_ptr)
{

    /* Determine if there is anything on the queue.  */
    if (nx_driver_information.nx_driver_transmit_queue_tail)
    {
    
        /* Yes, something is on the transmit queue. Simply add the new packet to the 
           tail.  */
        nx_driver_information.nx_driver_transmit_queue_tail -> nx_packet_queue_next  =  packet_ptr;

        /* Update the tail pointer.  */
        nx_driver_information.nx_driver_transmit_queue_tail =  packet_ptr;
    }
    else
    {
    
        /* First packet on the transmit queue.  */
        
        /* Setup head pointers.  */
        nx_driver_information.nx_driver_transmit_queue_head =  packet_ptr;
        nx_driver_information.nx_driver_transmit_queue_tail =  packet_ptr;
    
        /* Set the packet's next pointer to NULL.  */
        packet_ptr -> nx_packet_queue_next =  NX_NULL;
    }

    /* Increment the total packets queued.  */
    nx_driver_information.nx_driver_transmit_packets_queued++;
    
    /* Determine if the total packet queued exceeds the driver's maximum transmit
       queue depth.  */
    if (nx_driver_information.nx_driver_transmit_packets_queued > NX_DRIVER_MAX_TRANSMIT_QUEUE_DEPTH)
    {
    
        /* Yes, remove the head packet (oldest) packet in the transmit queue and release it.  */
        packet_ptr =  nx_driver_information.nx_driver_transmit_queue_head;

        /* Adjust the head pointer to the next packet.  */
        nx_driver_information.nx_driver_transmit_queue_head =  packet_ptr -> nx_packet_queue_next;

        /* Decrement the transmit packet queued count.  */
        nx_driver_information.nx_driver_transmit_packets_queued--;

        /* Remove the ethernet header.  */        
        NX_DRIVER_ETHERNET_HEADER_REMOVE(packet_ptr);

        /* Release the packet.  */
        nx_packet_transmit_release(packet_ptr);
    }
}


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _nx_driver_transmit_packet_dequeue                  PORTABLE C      */ 
/*                                                           5.0          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    Yuxin Zhou, Microsoft Corporation                                   */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This function removes the oldest transmit packet when the hardware  */ 
/*    transmit queue has new resources (usually after a transmit complete */ 
/*    interrupt) to send the packet. If there are no packets in the       */ 
/*    transmit queue, a NULL is returned.                                 */ 
/*                                                                        */ 
/*    Note: that it is assumed further driver interrupts are locked out   */ 
/*    during the call to this driver utility.                             */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    packet_ptr                            Packet pointer                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    _nx_driver_hardware_packet_send       Driver packet send function   */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  02-01-2018     Yuxin Zhou               Initial Version 5.0           */ 
/*                                                                        */ 
/**************************************************************************/ 
static NX_PACKET *_nx_driver_transmit_packet_dequeue(VOID)
{

NX_PACKET   *packet_ptr;


    /* Pickup the head pointer of the tranmit packet queue.  */
    packet_ptr =  nx_driver_information.nx_driver_transmit_queue_head;
    
    /* Determine if there is anything on the queue.  */
    if (packet_ptr)
    {
    
        /* Yes, something is on the transmit queue. Simply the packet from the head of the queue.  */

        /* Update the head pointer.  */
        nx_driver_information.nx_driver_transmit_queue_head =  packet_ptr -> nx_packet_queue_next;

        /* Clear the next pointer in the packet.  */
        packet_ptr -> nx_packet_queue_next =  NX_NULL;

        /* Decrement the transmit packet queued count.  */
        nx_driver_information.nx_driver_transmit_packets_queued--;
    }

    /* Return the packet pointer - NULL if there are no packets queued.  */
    return(packet_ptr);
}

#endif


void delay(void);
VOID nx_driver_link_mode_changed(VOID);
VOID nx_driver_gd_ethernet_isr(VOID);


void delay(void)
{
	volatile uint32_t i = 0;
	for (i = 0; i < 1000000; ++i)
	{
		__asm("NOP"); /* delay */
	}
}



/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _nx_driver_hardware_initialize                      PORTABLE C      */ 
/*                                                           5.0          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    Yuxin Zhou, Microsoft Corporation                                   */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This function processes hardware-specific initialization.           */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    driver_req_ptr                        Driver request pointer        */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    status                                [NX_SUCCESS|NX_DRIVER_ERROR]  */
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    ETH_BSP_Config                        Configure Ethernet            */ 
/*    ETH_MACAddressConfig                  Setup MAC address             */ 
/*    ETH_DMARxDescReceiveITConfig          Enable receive descriptors    */ 
/*    nx_packet_allocate                    Allocate receive packet(s)    */ 
/*                                                                        */
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    _nx_driver_initialize                 Driver initialize processing  */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  02-01-2018     Yuxin Zhou               Initial Version 5.0           */ 
/*                                                                        */ 
/**************************************************************************/ 
static UINT  _nx_driver_hardware_initialize(NX_IP_DRIVER *driver_req_ptr)
{
  
NX_PACKET           *packet_ptr;
UINT                i;
       
    /* Default to successful return.  */
    driver_req_ptr -> nx_ip_driver_status =  NX_SUCCESS;

    /* Setup indices.  */
    nx_driver_information.nx_driver_information_receive_current_index = 0; 
    nx_driver_information.nx_driver_information_transmit_current_index = 0; 
    nx_driver_information.nx_driver_information_transmit_release_index = 0;

    /* Clear the number of buffers in use counter.  */
    nx_driver_information.nx_driver_information_number_of_transmit_buffers_in_use = 0;
  
    /* Make sure there are receive packets... otherwise, return an error.  */
    if (nx_driver_information.nx_driver_information_packet_pool_ptr == NULL)
    {
    
        /* There must be receive packets. If not, return an error!  */
        return(NX_DRIVER_ERROR);
    }

    enet_system_setup();

    /* initialize MAC address in ethernet MAC */
    enet_mac_address_set(ENET_MAC_ADDRESS0, _nx_driver_hardware_address);
   
    /* Initialize TX Descriptors list: Ring Mode.  */
    /* Make sure Number of Buffer Descriptors is power of 2 */
#if (NX_DRIVER_TX_DESCRIPTORS & (NX_DRIVER_TX_DESCRIPTORS - 1)) != 0
#error "Number of Buffer Descriptors must be power of 2"
#endif
    
    nx_driver_information.nx_driver_information_dma_tx_descriptors = (enet_descriptors_struct*)(((UINT)nx_driver_information.nx_driver_information_dma_tx_descriptors_area + 15) & (~15));
   
    /* Fill each DMATxDesc descriptor with the right values.  */   
    for(i = 0; i < NX_DRIVER_TX_DESCRIPTORS; i++)
    {

        /* Initialize tx descriptors.  */
        nx_driver_information.nx_driver_information_dma_tx_descriptors[i].status = 0;
        nx_driver_information.nx_driver_information_dma_tx_descriptors[i].control_buffer_size = 0;
        nx_driver_information.nx_driver_information_transmit_packets[i] = NX_NULL;
        nx_driver_information.nx_driver_information_dma_tx_descriptors[i].status |= ENET_TDES0_INTC;

#ifdef GD_CHECKSUM_OFFLOAD
    /* enable the TCP, UDP and ICMP checksum insertion for the Tx frames */
        nx_driver_information.nx_driver_information_dma_tx_descriptors[i].status &= ~ENET_TDES0_CM;
        nx_driver_information.nx_driver_information_dma_tx_descriptors[i].status |= ENET_CHECKSUM_TCPUDPICMP_FULL;
#endif /* GD_CHECKSUM_OFFLOAD */

    }

    /* Put the Wrap indicaiton on the last descriptor.  */
    nx_driver_information.nx_driver_information_dma_tx_descriptors[NX_DRIVER_TX_DESCRIPTORS - 1].status |= ENET_TDES0_TERM;
    /* Set Transmit Descriptor List Address Register */
    ENET_DMA_TDTADDR = (ULONG) nx_driver_information.nx_driver_information_dma_tx_descriptors;



    /* Initialize RX Descriptors list: Ring Mode  */
    /* Make sure Number of Buffer Descriptors is power of 2 */
#if (NX_DRIVER_RX_DESCRIPTORS & (NX_DRIVER_RX_DESCRIPTORS - 1)) != 0
#error "Number of Buffer Descriptors must be power of 2"
#endif
    
    nx_driver_information.nx_driver_information_dma_rx_descriptors = (enet_descriptors_struct*)(((UINT)nx_driver_information.nx_driver_information_dma_rx_descriptors_area + 15) & (~15));
    
    /* Fill each DMARxDesc descriptor with the right values */  
    for(i = 0; i < NX_DRIVER_RX_DESCRIPTORS; i++)
    {

        /* Allocate a packet for the receive buffers.  */
        if (nx_packet_allocate(nx_driver_information.nx_driver_information_packet_pool_ptr, &packet_ptr, 
                               NX_RECEIVE_PACKET, NX_NO_WAIT) == NX_SUCCESS)
        {

            nx_driver_information.nx_driver_information_dma_rx_descriptors[i].status = ENET_RDES0_DAV;

            /* Adjust the packet. After remove enet frame, make sure the IP packet that transfered to the net 4byte alignment*/
            packet_ptr -> nx_packet_prepend_ptr += 2;

            nx_driver_information.nx_driver_information_dma_rx_descriptors[i].buffer1_addr = (ULONG)packet_ptr->nx_packet_prepend_ptr;
            nx_driver_information.nx_driver_information_receive_packets[i] = packet_ptr;
            nx_driver_information.nx_driver_information_rx_buffer_size = packet_ptr -> nx_packet_data_end - packet_ptr -> nx_packet_data_start;
            nx_driver_information.nx_driver_information_dma_rx_descriptors[i].control_buffer_size = nx_driver_information.nx_driver_information_rx_buffer_size;

        }
        else
        {

            /* Cannot allocate packets from the packet pool. */
            return(NX_DRIVER_ERROR);
        }

    }
    
    /* Put the Wrap indicaiton on the last descriptor.  */

    /* Save the size of one rx buffer.  */
    nx_driver_information.nx_driver_information_rx_buffer_size = packet_ptr -> nx_packet_data_end - packet_ptr -> nx_packet_data_start;
        
    /* Configure the Receive Buffer Size Register.  */
    nx_driver_information.nx_driver_information_dma_rx_descriptors[NX_DRIVER_RX_DESCRIPTORS - 1].control_buffer_size = nx_driver_information.nx_driver_information_rx_buffer_size;
    nx_driver_information.nx_driver_information_dma_rx_descriptors[NX_DRIVER_RX_DESCRIPTORS - 1].control_buffer_size |= ENET_RDES1_RERM;

    /* Set Receive Descriptor List Address Register.  */
    ENET_DMA_RDTADDR = (ULONG) nx_driver_information.nx_driver_information_dma_rx_descriptors;

    for (i = 64; i; i--)
    {
        
        nx_driver_information.nx_driver_information_multicast_count[i] = 0;
    }
    
    /* Return success!  */
    return(NX_SUCCESS);
} 


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _nx_driver_hardware_enable                          PORTABLE C      */ 
/*                                                           5.0          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    Yuxin Zhou, Microsoft Corporation                                   */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This function processes hardware-specific link enable requests.     */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    driver_req_ptr                        Driver request pointer        */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    status                                [NX_SUCCESS|NX_DRIVER_ERROR]  */
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 

/*                                                                        */
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    _nx_driver_enable                     Driver link enable processing */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  02-01-2018     Yuxin Zhou               Initial Version 5.0           */ 
/*                                                                        */ 
/**************************************************************************/ 
static UINT  _nx_driver_hardware_enable(NX_IP_DRIVER *driver_req_ptr)
{
    /* Enable Ethernet interrupt.  */
    enet_interrupt_enable(ENET_DMA_INT_NIE);
    enet_interrupt_enable(ENET_DMA_INT_RIE);
    enet_interrupt_enable(ENET_DMA_INT_TIE);

    /* Start Ethernet.  */
    enet_enable();

    /* Return success!  */
    return(NX_SUCCESS);
}


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _nx_driver_hardware_disable                         PORTABLE C      */ 
/*                                                           5.0          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    Yuxin Zhou, Microsoft Corporation                                   */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This function processes hardware-specific link disable requests.    */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    driver_req_ptr                        Driver request pointer        */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    status                                [NX_SUCCESS|NX_DRIVER_ERROR]  */
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 

/*                                                                        */
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    _nx_driver_disable                    Driver link disable processing*/ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  02-01-2018     Yuxin Zhou               Initial Version 5.0           */ 
/*                                                                        */ 
/**************************************************************************/ 
static UINT  _nx_driver_hardware_disable(NX_IP_DRIVER *driver_req_ptr)
{
    /* Stop the Ethernet.  */
    enet_disable();

    /* Return success!  */
    return(NX_SUCCESS);
}


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _nx_driver_hardware_packet_send                     PORTABLE C      */ 
/*                                                           5.0          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    Yuxin Zhou, Microsoft Corporation                                   */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This function processes hardware-specific packet send requests.     */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    packet_ptr                            Pointer to packet to send     */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    status                                [NX_SUCCESS|NX_DRIVER_ERROR]  */
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    [_nx_driver_transmit_packet_enqueue]  Optional internal transmit    */ 
/*                                            packet queue routine        */ 
/*                                                                        */
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    _nx_driver_packet_send                Driver packet send processing */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  02-01-2018     Yuxin Zhou               Initial Version 5.0           */ 
/*                                                                        */ 
/**************************************************************************/ 
static UINT  _nx_driver_hardware_packet_send(NX_PACKET *packet_ptr)
{

ULONG          curIdx;
NX_PACKET      *pktIdx;
ULONG          bd_count = 0;
UCHAR          remainder = 0;
UCHAR*         src_addr;

    /* Pick up the first BD. */
    curIdx = nx_driver_information.nx_driver_information_transmit_current_index;
    
    /* Check if it is a free descriptor.  */
    if ((nx_driver_information.nx_driver_information_dma_tx_descriptors[curIdx].status & ENET_TDES0_DAV) || nx_driver_information.nx_driver_information_transmit_packets[curIdx])
    { 
        /* Buffer is still owned by device.  */
        return(NX_DRIVER_ERROR);
    }

    /* Set the buffer size.  */
    nx_driver_information.nx_driver_information_dma_tx_descriptors[curIdx].control_buffer_size = (packet_ptr -> nx_packet_append_ptr - packet_ptr->nx_packet_prepend_ptr);

    remainder = (UCHAR )((ULONG)(packet_ptr->nx_packet_prepend_ptr - 2)& 0x07);
 
    if(remainder)
    {
      src_addr = packet_ptr->nx_packet_prepend_ptr;
      
      /*make sure transmit BD buffer 8byte alignment*/
      packet_ptr->nx_packet_prepend_ptr -= remainder;
      
      memmove(packet_ptr->nx_packet_prepend_ptr,src_addr,nx_driver_information.nx_driver_information_dma_tx_descriptors[curIdx].control_buffer_size);
    }
    
    /* Find the Buffer, set the Buffer pointer. */
    nx_driver_information.nx_driver_information_dma_tx_descriptors[curIdx].buffer1_addr = (ULONG)(packet_ptr->nx_packet_prepend_ptr);

    /* Clear the first Descriptor's LS bit.  */
    nx_driver_information.nx_driver_information_dma_tx_descriptors[curIdx].status &= ~ENET_TDES0_LSG;

    /* Find next packet.  */
    for (pktIdx = packet_ptr -> nx_packet_next;
         pktIdx != NX_NULL;
         pktIdx = pktIdx -> nx_packet_next)
    {
        
        /* Move to next descriptor.  */
        curIdx = (curIdx + 1) & (NX_DRIVER_TX_DESCRIPTORS - 1);
    
        /* Check if it is a free descriptor.  */
        if ((nx_driver_information.nx_driver_information_dma_tx_descriptors[curIdx].status & ENET_TDES0_DAV) || nx_driver_information.nx_driver_information_transmit_packets[curIdx])
        {
            
            /* No more descriptor available, return driver error status.  */
            return(NX_DRIVER_ERROR);
        }
        

        /* Find the Buffer, set the Buffer pointer.  */
        nx_driver_information.nx_driver_information_dma_tx_descriptors[curIdx].buffer1_addr = (ULONG)(pktIdx->nx_packet_prepend_ptr);
    
        /* Set the buffer size.  */
        nx_driver_information.nx_driver_information_dma_tx_descriptors[curIdx].control_buffer_size = (pktIdx -> nx_packet_append_ptr - pktIdx->nx_packet_prepend_ptr);
        
        /* Clear the descriptor's LS bit.  */
        nx_driver_information.nx_driver_information_dma_tx_descriptors[curIdx].status &= ~ENET_TDES0_LSG;

        /* Increment the BD count.  */
        bd_count++;
        
    }
    
    /* Set the last Descriptor's LS & IC & OWN bit.  */
    nx_driver_information.nx_driver_information_dma_tx_descriptors[curIdx].status |= (ENET_TDES0_LSG | ENET_TDES0_DAV);

    /* Save the pkt pointer to release.  */
    nx_driver_information.nx_driver_information_transmit_packets[curIdx] = packet_ptr;
    
    /* Set the current index to the next descriptor.  */
    nx_driver_information.nx_driver_information_transmit_current_index = (curIdx + 1) & (NX_DRIVER_TX_DESCRIPTORS - 1);
    
    /* Increment the transmit buffers in use count.  */
    nx_driver_information.nx_driver_information_number_of_transmit_buffers_in_use += bd_count + 1;
    
    /* Set OWN bit to indicate BDs are ready.  */
    for (; bd_count > 0; bd_count--)
    {
        
        /* Set OWN bit in reverse order, move to prevous BD.  */
        curIdx = (curIdx - 1) & (NX_DRIVER_TX_DESCRIPTORS - 1);
        
        /* Set this BD's OWN bit.  */
        nx_driver_information.nx_driver_information_dma_tx_descriptors[curIdx].status |= ENET_TDES0_DAV;
    }

    /* check Tx buffer unavailable flag status */
    if (((uint32_t)RESET != (ENET_DMA_STAT & ENET_DMA_STAT_TBU)) || ((uint32_t)RESET != (ENET_DMA_STAT & ENET_DMA_STAT_TU))){
        /* clear TBU and TU flag */
        ENET_DMA_STAT = (ENET_DMA_STAT & ENET_DMA_STAT_TBU);
        ENET_DMA_STAT = (ENET_DMA_STAT & ENET_DMA_STAT_TU);
        /* resume DMA transmission by writing to the TPEN register*/
        ENET_DMA_TPEN = 0U;
    }

    return(NX_SUCCESS);
}


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*    nx_crc32()                                         PORTABLE C       */ 
/*                                                           1.0          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    SK - Microsoft Corporation                                          */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    Calculate 32bit CRC using reversed Poly for Ethernet.               */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    UCHAR dbuf[]  - pointer to the data buffer.                         */
/*    INT length    - Length of the data                                  */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    CRC value                                                           */
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    _nx_driver_hardware_multicast_join                                  */ 
/*    _nx_driver_hardware_multicast_leave                                 */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  03-09-2007           SK                    Initial Version 1.0        */ 
/*                                                                        */ 
/**************************************************************************/ 
static ULONG  nx_crc32(UCHAR dbuf[], INT length)    
{
    
INT             i;
INT             bit;
ULONG           crc  = 0xFFFFFFFFUL;
ULONG           poly = 0xEDB88320UL;
ULONG           p;
ULONG           data;


    for (i = 0; i < length; i++) 
    {
        data = ((ULONG)dbuf[i]);

        for (bit = 0; bit < 8; bit++) 
        {
            p   = (crc ^ ((ULONG)data)) & 1UL;
            crc >>= 1;
            if (p != 0)
                crc ^= poly;
            data >>= 1;
        }
    }
    
    return ~crc;
}   /* nx_crc32 */


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _nx_driver_hardware_multicast_join                  PORTABLE C      */ 
/*                                                           5.0          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    Yuxin Zhou, Microsoft Corporation                                   */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This function processes hardware-specific multicast join requests.  */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    driver_req_ptr                        Driver request pointer        */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    status                                [NX_SUCCESS|NX_DRIVER_ERROR]  */
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    _nx_driver_multicast_join             Driver multicast join         */ 
/*                                            processing                  */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  02-01-2018     Yuxin Zhou               Initial Version 5.0           */ 
/*                                                                        */ 
/**************************************************************************/ 
static UINT  _nx_driver_hardware_multicast_join(NX_IP_DRIVER *driver_req_ptr)
{

UCHAR           adr[NX_DRIVER_PHYSICAL_ADDRESS_SIZE] ;
INT             h_val;
ULONG           crc_val;


    /* Set up the array to pass to the hash_value function.  */
    adr[0] = (UCHAR) (driver_req_ptr ->nx_ip_driver_physical_address_msw >> 8);
    adr[1] = (UCHAR) (driver_req_ptr ->nx_ip_driver_physical_address_msw);
    adr[2] = (UCHAR) (driver_req_ptr ->nx_ip_driver_physical_address_lsw >> 24);
    adr[3] = (UCHAR) (driver_req_ptr ->nx_ip_driver_physical_address_lsw >> 16);
    adr[4] = (UCHAR) (driver_req_ptr ->nx_ip_driver_physical_address_lsw >> 8);
    adr[5] = (UCHAR) (driver_req_ptr ->nx_ip_driver_physical_address_lsw);

    /* Get the CRC done.  */
    crc_val = nx_crc32(adr, NX_DRIVER_PHYSICAL_ADDRESS_SIZE);

    /* Use only 6 MSbs to obtain value in range 0..63.  */
    crc_val >>= (32 - 6);          
    h_val = 63 - (int) crc_val ;      /* pointer to 48 bit address */

    if(nx_driver_information.nx_driver_information_multicast_count[h_val] == 255)
    {
        
        return NX_NO_MORE_ENTRIES;
    }
    
    nx_driver_information.nx_driver_information_multicast_count[h_val]++;
    
    if (h_val < 32)
        ENET_MAC_HLL |= 1 <<  h_val;
    else
        ENET_MAC_HLH |= 1 << (h_val - 32);
        
    /* Return success.  */
    return(NX_SUCCESS);
}


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _nx_driver_hardware_multicast_leave                 PORTABLE C      */ 
/*                                                           5.0          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    Yuxin Zhou, Microsoft Corporation                                   */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This function processes hardware-specific multicast leave requests. */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    driver_req_ptr                        Driver request pointer        */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    status                                [NX_SUCCESS|NX_DRIVER_ERROR]  */
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    _nx_driver_multicast_leave            Driver multicast leave        */ 
/*                                            processing                  */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  02-01-2018     Yuxin Zhou               Initial Version 5.0           */ 
/*                                                                        */ 
/**************************************************************************/ 
static UINT  _nx_driver_hardware_multicast_leave(NX_IP_DRIVER *driver_req_ptr)
{
    
UCHAR           adr[NX_DRIVER_PHYSICAL_ADDRESS_SIZE] ;
INT             h_val;
ULONG           crc_val;


    /* Set up the array to pass to the hash_value function.  */
    adr[0] = (UCHAR) (driver_req_ptr ->nx_ip_driver_physical_address_msw >> 8);
    adr[1] = (UCHAR) (driver_req_ptr ->nx_ip_driver_physical_address_msw);
    adr[2] = (UCHAR) (driver_req_ptr ->nx_ip_driver_physical_address_lsw >> 24);
    adr[3] = (UCHAR) (driver_req_ptr ->nx_ip_driver_physical_address_lsw >> 16);
    adr[4] = (UCHAR) (driver_req_ptr ->nx_ip_driver_physical_address_lsw >> 8);
    adr[5] = (UCHAR) (driver_req_ptr ->nx_ip_driver_physical_address_lsw);

    /* Get the CRC done.  */
    crc_val = nx_crc32(adr, NX_DRIVER_PHYSICAL_ADDRESS_SIZE);

    /* Use only 6 MSbs to obtain value in range 0..63.  */
    crc_val >>= (32 - 6);          
    h_val = 63 - (int) crc_val ;      /* pointer to 48 bit address */
    
    if(nx_driver_information.nx_driver_information_multicast_count[h_val] == 1)
    {
        
        if (h_val < 32)
            ENET_MAC_HLL &= ~(1 <<  h_val);
        else
            ENET_MAC_HLH &= ~(1 << (h_val - 32));
    }
    else if(nx_driver_information.nx_driver_information_multicast_count[h_val] == 0)
    {
        
        return NX_NOT_SUCCESSFUL;
    }

    nx_driver_information.nx_driver_information_multicast_count[h_val]--;

    /* Return success.  */
    return(NX_SUCCESS);
}


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _nx_driver_hardware_get_status                      PORTABLE C      */ 
/*                                                           5.0          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    Yuxin Zhou, Microsoft Corporation                                   */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This function processes hardware-specific get status requests.      */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    driver_req_ptr                        Driver request pointer        */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    status                                [NX_SUCCESS|NX_DRIVER_ERROR]  */
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    _nx_driver_get_status                 Driver get status processing  */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  02-01-2018     Yuxin Zhou               Initial Version 5.0           */ 
/*                                                                        */ 
/**************************************************************************/ 
static UINT  _nx_driver_hardware_get_status(NX_IP_DRIVER *driver_req_ptr)
{

    *(driver_req_ptr -> nx_ip_driver_return_ptr) = driver_req_ptr -> nx_ip_driver_interface -> nx_interface_link_up;
    /* Return success.  */
    return(NX_SUCCESS);
}


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _nx_driver_hardware_packet_transmitted              PORTABLE C      */ 
/*                                                           5.0          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    Yuxin Zhou, Microsoft Corporation                                   */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This function processes packets transmitted by the ethernet         */ 
/*    controller.                                                         */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    None                                                                */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    nx_packet_transmit_release            Release transmitted packet    */ 
/*    [_nx_driver_transmit_packet_dequeue]  Optional transmit packet      */ 
/*                                            dequeue                     */ 
/*                                                                        */
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    _nx_driver_deferred_processing        Deferred driver processing    */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  02-01-2018     Yuxin Zhou               Initial Version 5.0           */ 
/*                                                                        */ 
/**************************************************************************/ 
static VOID  _nx_driver_hardware_packet_transmitted(VOID)
{

ULONG numOfBuf =  nx_driver_information.nx_driver_information_number_of_transmit_buffers_in_use;
ULONG idx =       nx_driver_information.nx_driver_information_transmit_release_index;
    
    
    /* Loop through buffers in use.  */
    while (numOfBuf--)
    {

        /* If no packet, just examine the next packet.  */
        if (nx_driver_information.nx_driver_information_transmit_packets[idx] == NX_NULL) 
        {
        
            /* No packet in use, skip to next.  */
            idx = (idx + 1) & (NX_DRIVER_TX_DESCRIPTORS - 1);
            continue;
        }
        
        /* Determine if the packet has been transmitted.  */
        if ((nx_driver_information.nx_driver_information_dma_tx_descriptors[idx].status & ENET_TDES0_DAV) == 0)
        {

            /* Yes, packet has been transmitted.  */
            
            /* Remove the Ethernet header and release the packet.  */
            NX_DRIVER_ETHERNET_HEADER_REMOVE(nx_driver_information.nx_driver_information_transmit_packets[idx]);

            /* Release the packet.  */
            nx_packet_transmit_release(nx_driver_information.nx_driver_information_transmit_packets[idx]);

            /* Clear the entry in the in-use array.  */
            nx_driver_information.nx_driver_information_transmit_packets[idx] = NX_NULL;
            
            /* Update the transmit relesae index and number of buffers in use.  */
            idx = (idx + 1) & (NX_DRIVER_TX_DESCRIPTORS - 1);
            nx_driver_information.nx_driver_information_number_of_transmit_buffers_in_use = numOfBuf;
            nx_driver_information.nx_driver_information_transmit_release_index = idx;
        }
        else
        {

            /* Get out of the loop!  */
            break;
        }
    }
}


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _nx_driver_hardware_packet_received                 PORTABLE C      */ 
/*                                                           5.0          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    Yuxin Zhou, Microsoft Corporation                                   */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This function processes packets received by the ethernet            */ 
/*    controller.                                                         */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    None                                                                */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _nx_driver_transfer_to_netx           Transfer packet to NetX       */ 
/*    nx_packet_allocate                    Allocate receive packets      */ 
/*    nx_packet_release                     Release receive packets       */
/*                                                                        */
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    _nx_driver_deferred_processing        Deferred driver processing    */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  02-01-2018     Yuxin Zhou               Initial Version 5.0           */ 
/*                                                                        */ 
/**************************************************************************/ 
static VOID  _nx_driver_hardware_packet_received(VOID)
{

NX_PACKET     *packet_ptr;
ULONG          bd_count = 0;
INT            i;
ULONG          idx;
ULONG          temp_idx;
ULONG          first_idx = nx_driver_information.nx_driver_information_receive_current_index;
NX_PACKET     *received_packet_ptr = nx_driver_information.nx_driver_information_receive_packets[first_idx];


    /* Find out the BDs that owned by CPU.  */
    for (first_idx = idx = nx_driver_information.nx_driver_information_receive_current_index;
        (nx_driver_information.nx_driver_information_dma_rx_descriptors[idx].status & ENET_RDES0_DAV) == 0;
         idx = (idx + 1) & (NX_DRIVER_RX_DESCRIPTORS - 1))
    {
        
        /* Is the BD marked as the end of a frame?  */
        if (nx_driver_information.nx_driver_information_dma_rx_descriptors[idx].status & ENET_RDES0_LDES)
        {
            
            /* Yes, this BD is the last BD in the frame, set the last NX_PACKET's nx_packet_next to NULL.  */
            nx_driver_information.nx_driver_information_receive_packets[idx] -> nx_packet_next = NX_NULL;
            
            /* Store the length of the packet in the first NX_PACKET.  */

            nx_driver_information.nx_driver_information_receive_packets[first_idx] -> nx_packet_length = (nx_driver_information.nx_driver_information_dma_rx_descriptors[idx].control_buffer_size);

            /* Adjust nx_packet_append_ptr with the size of the data in this buffer.  */
            nx_driver_information.nx_driver_information_receive_packets[idx] -> nx_packet_append_ptr = nx_driver_information.nx_driver_information_receive_packets[idx]->nx_packet_prepend_ptr
                                                                                                     + nx_driver_information.nx_driver_information_receive_packets[first_idx]->nx_packet_length
                                                                                                     - bd_count * nx_driver_information.nx_driver_information_rx_buffer_size
                                                                                                     + (bd_count > 0 ? 2 : 0);

            /* Allocate new NX_PACKETs for BDs.  */
            for (i = bd_count; i >= 0; i--)
            {
                
                temp_idx = (first_idx + i) & (NX_DRIVER_RX_DESCRIPTORS - 1);
                
                /* Allocate a new packet from the packet pool.  */
                if (nx_packet_allocate(nx_driver_information.nx_driver_information_packet_pool_ptr, &packet_ptr, 
                                          NX_RECEIVE_PACKET, NX_NO_WAIT) == NX_SUCCESS)
                {

                    /* Adjust the new packet. After remove enet frame, make sure the IP packet that transfered to the net 4byte alignment*/
                    packet_ptr -> nx_packet_prepend_ptr += 2;

                    nx_driver_information.nx_driver_information_dma_rx_descriptors[temp_idx].buffer1_addr = (uint32_t)packet_ptr->nx_packet_prepend_ptr;
                    nx_driver_information.nx_driver_information_dma_rx_descriptors[temp_idx].status |= ENET_RDES0_DAV;
                    nx_driver_information.nx_driver_information_receive_packets[temp_idx] = packet_ptr;
                }
                else
                {
                    
                    /* Allocation failed, get out of the loop.  */
                    break;
                }
            }
            
            if (i >= 0)
            {
                
                /* At least one packet allocation was failed, release the received packet.  */
                nx_packet_release(nx_driver_information.nx_driver_information_receive_packets[temp_idx] -> nx_packet_next);
                
                for (; i >= 0; i--)
                {
                    
                    /* Free up the BD to ready state. */
                    temp_idx = (first_idx + i) & (NX_DRIVER_RX_DESCRIPTORS - 1);
                    nx_driver_information.nx_driver_information_dma_rx_descriptors[temp_idx].status |= ENET_RDES0_DAV;
                    nx_driver_information.nx_driver_information_receive_packets[temp_idx] -> nx_packet_prepend_ptr = nx_driver_information.nx_driver_information_receive_packets[temp_idx] -> nx_packet_data_start;
                }
            }
            else
            {

                /* Transfer the packet to NetX.  */
                _nx_driver_transfer_to_netx(nx_driver_information.nx_driver_information_ip_ptr, received_packet_ptr);
            }
            
            /* Set the first BD index for the next packet.  */
            first_idx = (idx + 1) & (NX_DRIVER_RX_DESCRIPTORS - 1);
            
            /* Update the current receive index.  */
            nx_driver_information.nx_driver_information_receive_current_index = first_idx;
            
            received_packet_ptr = nx_driver_information.nx_driver_information_receive_packets[first_idx];
            
            bd_count = 0;

        }
        else
        {
            
            /* This BD is not the last BD of a frame. It is a intermediate descriptor.  */
            
            nx_driver_information.nx_driver_information_receive_packets[idx] -> nx_packet_next = nx_driver_information.nx_driver_information_receive_packets[(idx + 1) & (NX_DRIVER_RX_DESCRIPTORS - 1)];
            
            nx_driver_information.nx_driver_information_receive_packets[idx] -> nx_packet_append_ptr = nx_driver_information.nx_driver_information_receive_packets[idx] -> nx_packet_data_end;
 
            bd_count++;
        }
    }

    /* check Rx buffer unavailable flag status */
    if ((uint32_t)RESET != (ENET_DMA_STAT & ENET_DMA_STAT_RBU)){
        /* clear RBU flag */
        ENET_DMA_STAT = ENET_DMA_STAT_RBU;
        /* resume DMA reception by writing to the RPEN register*/
        ENET_DMA_RPEN = 0U;
    }

}

/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    nx_driver_link_mode_changed                         PORTABLE C      */ 
/*                                                           5.0          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    Yuxin Zhou, Microsoft Corporation                                   */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This function changes the link mode of the Ethernet.                */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    None                                                                */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    enet_duplex                           Set duplex mode               */ 
/*    nx_packet_transmit_release            Release the packet            */ 
/*                                                                        */
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    nx_driver_ethernet_phy_isr                                          */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  02-01-2018     Yuxin Zhou               Initial Version 5.0           */ 
/*                                                                        */ 
/**************************************************************************/ 
VOID  nx_driver_link_mode_changed(VOID)
{

ULONG numOfBuf;
ULONG idx;


    /* Stop the Ethernet.  */
    enet_disable();
    
    /* Set speed for RMII mode.  */
    if (nx_driver_information.nx_driver_information_link_speed == PHY_SPEED_STATUS)
    {
        
        ENET_MAC_CFG &= ~ENET_MAC_CFG_SPD;
    }
    else
    {
        
        ENET_MAC_CFG |= ENET_MAC_CFG_SPD;
    }
    
    /* Set duplex mode.  */
    /* Set the duplex on the selected FEC controller*/
    if (nx_driver_information.nx_driver_information_link_duplex == PHY_DUPLEX_STATUS)
    {

        ENET_MAC_CFG &= ~ENET_MAC_CFG_DPM;
    }
    else
    {

        ENET_MAC_CFG |= ENET_MAC_CFG_DPM;
    }
    
    if (nx_driver_information.nx_driver_information_state >= NX_DRIVER_STATE_INITIALIZED)
    {
        
        numOfBuf = nx_driver_information.nx_driver_information_number_of_transmit_buffers_in_use;
        idx =      nx_driver_information.nx_driver_information_transmit_release_index;

        /* Reset indices.  */
        nx_driver_information.nx_driver_information_receive_current_index = 0; 
        nx_driver_information.nx_driver_information_transmit_current_index = 0; 
        nx_driver_information.nx_driver_information_transmit_release_index = 0;
        nx_driver_information.nx_driver_information_number_of_transmit_buffers_in_use = 0;
        
        /* Release transmit packets if any.  */
        while (numOfBuf--)
        {
            
            /* If no packet, just examine the next packet.  */
            if (nx_driver_information.nx_driver_information_transmit_packets[idx] == NX_NULL) 
            {
            
                /* No packet in use, skip to next.  */
                idx = (idx + 1) & (NX_DRIVER_TX_DESCRIPTORS - 1);
                continue;
            }           
            
            /* Remove the Ethernet header and release the packet.  */
            NX_DRIVER_ETHERNET_HEADER_REMOVE(nx_driver_information.nx_driver_information_transmit_packets[idx]);

            /* Release the packet.  */
            nx_packet_transmit_release(nx_driver_information.nx_driver_information_transmit_packets[idx]);
        }
        
        /* Free receive descriptors.  */
        for (idx = 0; idx < NX_DRIVER_RX_DESCRIPTORS; idx++)
        {
            
            nx_driver_information.nx_driver_information_dma_rx_descriptors[idx].status |= ENET_RDES0_LDES;
        }
    }
    
    /* Set Transmit Descriptor List Address Register */
    ENET_DMA_TDTADDR = (ULONG) nx_driver_information.nx_driver_information_dma_tx_descriptors;
    
    /* Configure the Receive Buffer Size Register.  */
    nx_driver_information.nx_driver_information_dma_rx_descriptors[NX_DRIVER_RX_DESCRIPTORS - 1].control_buffer_size = nx_driver_information.nx_driver_information_rx_buffer_size;
    
    /* Set Receive Descriptor List Address Register.  */
    ENET_DMA_RDTADDR = (ULONG) nx_driver_information.nx_driver_information_dma_rx_descriptors;

    if (nx_driver_information.nx_driver_information_state >= NX_DRIVER_STATE_LINK_ENABLED)
    {
        
        /* Enable ethernet & start packet receiving.  */

        enet_enable();
        ENET_DMA_RPEN = 0U;
    }
}

/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    nx_driver_gd_ethernet_isr                          PORTABLE C      */ 
/*                                                           5.0          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    Yuxin Zhou, Microsoft Corporation                                   */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This function processing incoming packets.  This routine is         */
/*    be called from the receive packet ISR and assumes that the          */ 
/*    interrupt is saved/restored around the call by ThreadX.             */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    None                                                                */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _nx_ip_driver_deferred_processing     IP receive packet processing  */ 
/*                                                                        */
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    ISR                                                                 */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  02-01-2018     Yuxin Zhou               Initial Version 5.0           */ 
/*                                                                        */ 
/**************************************************************************/ 
VOID  nx_driver_gd_ethernet_isr(VOID)
{
UINT status;
    status = ENET_DMA_STAT;

    if(status & ENET_DMA_STAT_RS )
    {
    /* Receive packet interrupt.  */
#ifdef NX_DRIVER_ENABLE_DEFERRED

    /* Set the receive packet interrupt.  */
    nx_driver_information.nx_driver_information_deferred_events |= NX_DRIVER_DEFERRED_PACKET_RECEIVED;
#else

    /* Process received packet(s).  */
    _nx_driver_hardware_packet_received();
#endif

#ifdef NX_DRIVER_ENABLE_DEFERRED

    /* Call NetX deferred driver processing.  */        
    _nx_ip_driver_deferred_processing(nx_driver_information.nx_driver_information_ip_ptr);
#endif

    /* clear the enet DMA Rx interrupt pending bits */
    enet_interrupt_flag_clear(ENET_DMA_INT_FLAG_RS_CLR);
    enet_interrupt_flag_clear(ENET_DMA_INT_FLAG_NI_CLR);
    }

    if(status & ENET_DMA_STAT_TS)
    {

      /* resume DMA transmission by writing to the TPEN register*/
      ENET_DMA_TPEN = 0U;
    
#ifdef NX_DRIVER_ENABLE_DEFERRED

    /* Set the transmit complete bit.  */
    nx_driver_information.nx_driver_information_deferred_events |= NX_DRIVER_DEFERRED_PACKET_TRANSMITTED;
#else

    /* Process transmitted packet(s).  */
    _nx_driver_hardware_packet_transmitted();
#endif

#ifdef NX_DRIVER_ENABLE_DEFERRED

    /* Call NetX deferred driver processing.  */        
    _nx_ip_driver_deferred_processing(nx_driver_information.nx_driver_information_ip_ptr);
#endif

    /* clear the enet DMA Tx interrupt pending bits */
    enet_interrupt_flag_clear(ENET_DMA_INT_FLAG_TS_CLR);
    enet_interrupt_flag_clear(ENET_DMA_INT_FLAG_NI_CLR);
    }
}

/****** DRIVER SPECIFIC ****** Start of part/vendor specific internal driver functions.  */
