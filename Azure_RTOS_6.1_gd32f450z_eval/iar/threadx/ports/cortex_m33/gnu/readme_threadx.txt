                     Microsoft's Azure RTOS ThreadX for Cortex-M33 

                              Using the GNU Tools

1.  Building the ThreadX run-time Library

Import all ThreadX common and port-specific source files into a GNU project.
Configure the project to build a library rather than an executable. This 
results in the ThreadX run-time library file tx.a, which is needed by 
the application.


2.  Demonstration System

No demonstration project is provided. 


3.  System Initialization

The entry point in ThreadX for the Cortex-M33 using gnu tools uses the standard GNU 
Cortex-M33 reset sequence. From the reset vector the C runtime will be initialized.

The ThreadX tx_initialize_low_level.S file is responsible for setting up 
various system data structures, the vector area, and a periodic timer interrupt 
source. 

In addition, _tx_initialize_low_level determines the first available 
address for use by the application, which is supplied as the sole input 
parameter to your application definition function, tx_application_define.


4.  Register Usage and Stack Frames

The following defines the saved context stack frames for context switches
that occur as a result of interrupt handling or from thread-level API calls.
All suspended threads have the same stack frame in the Cortex-M33 version of
ThreadX. The top of the suspended thread's stack is pointed to by 
tx_thread_stack_ptr in the associated thread control block TX_THREAD.

Non-FPU Stack Frame:

  Stack Offset     Stack Contents 

     0x00               LR          Interrupted LR (LR at time of PENDSV)
     0x04               r4
     0x08               r5
     0x0C               r6
     0x10               r7
     0x14               r8
     0x18               r9
     0x1C               r10
     0x20               r11
     0x24               r0          (Hardware stack starts here!!)
     0x28               r1
     0x2C               r2
     0x30               r3
     0x34               r12
     0x38               lr
     0x3C               pc
     0x40               xPSR

FPU Stack Frame (only interrupted thread with FPU enabled):

  Stack Offset     Stack Contents 

     0x00               LR          Interrupted LR (LR at time of PENDSV)
     0x04               s0
     0x08               s1
     0x0C               s2
     0x10               s3
     0x14               s4
     0x18               s5
     0x1C               s6
     0x20               s7
     0x24               s8
     0x28               s9
     0x2C               s10
     0x30               s11
     0x34               s12
     0x38               s13
     0x3C               s14
     0x40               s15
     0x44               s16
     0x48               s17
     0x4C               s18
     0x50               s19
     0x54               s20
     0x58               s21
     0x5C               s22
     0x60               s23
     0x64               s24
     0x68               s25
     0x6C               s26
     0x70               s27
     0x74               s28
     0x78               s29
     0x7C               s30
     0x80               s31
     0x84               fpscr
     0x88               r4
     0x8C               r5
     0x90               r6
     0x94               r7
     0x98               r8
     0x9C               r9
     0xA0               r10
     0xA4               r11
     0xA8               r0          (Hardware stack starts here!!)
     0xAC               r1
     0xB0               r2
     0xB4               r3
     0xB8               r12
     0xBC               lr
     0xC0               pc
     0xC4               xPSR


5.  Improving Performance

To make ThreadX and the application(s) run faster, you can enable 
all compiler optimizations. 

In addition, you can eliminate the ThreadX basic API error checking by 
compiling your application code with the symbol TX_DISABLE_ERROR_CHECKING 
defined. 


6.  Interrupt Handling

ThreadX provides complete and high-performance interrupt handling for Cortex-M33
targets. There are a certain set of requirements that are defined in the 
following sub-sections:


6.1  Vector Area

The Cortex-M33 vectors start at the label __tx_vectors or similar. The application may modify
the vector area according to its needs. There is code in tx_initialize_low_level() that will 
configure the vector base register. 


6.2 Managed Interrupts

ISRs can be written completely in C (or assembly language) without any calls to
_tx_thread_context_save or _tx_thread_context_restore. These ISRs are allowed access to the
ThreadX API that is available to ISRs.

ISRs written in C will take the form (where "your_C_isr" is an entry in the vector table):

void    your_C_isr(void)
{

    /* ISR processing goes here, including any needed function calls.  */
}

ISRs written in assembly language will take the form:


    .global  your_assembly_isr
    .thumb_func
your_assembly_isr:
    PUSH    {r0, lr}
;
;    /* Do interrupt handler work here */
;    /* BL <your interrupt routine in C> */

    POP     {r0, lr}
    BX      lr

Note: the Cortex-M33 requires exception handlers to be thumb labels, this implies bit 0 set.
To accomplish this, the declaration of the label has to be preceded by the assembler directive
.thumb_func to instruct the linker to create thumb labels. The label __tx_IntHandler needs to 
be inserted in the correct location in the interrupt vector table. This table is typically 
located in either your runtime startup file or in the tx_initialize_low_level.S file.


7. FPU Support

ThreadX for Cortex-M33 supports automatic ("lazy") VFP support, which means that applications threads 
can simply use the VFP and ThreadX automatically maintains the VFP registers as part of the thread 
context.


8.  Revision History

For generic code revision information, please refer to the readme_threadx_generic.txt
file, which is included in your distribution. The following details the revision
information associated with this specific port of ThreadX:

09-30-2020  Initial ThreadX 6.1 version for Cortex-M33 using GNU tools.


Copyright(c) 1996-2020 Microsoft Corporation


https://azure.com/rtos

