;********************************************************************************
;* File Name          : stm32wbaxx_ResetHandler.s
;* Author             : MCD Application Team
;* Description        : STM32WBA5xx Ultra Low Power Devices specific
;                       Reset handler for connectivity applications.
;                       EWARM toolchain.
;********************************************************************************
;* @attention
;*
;* Copyright (c) 2022-2025 STMicroelectronics.
;* All rights reserved.
;*
;* This software is licensed under terms that can be found in the LICENSE file
;* in the root directory of this software component.
;* If no LICENSE file comes with this software, it is provided AS-IS.
;*
;*******************************************************************************
;
; Cortex-M version
;
        EXTERN  __iar_program_start
        EXTERN  SystemInit
        EXTERN  is_boot_from_standby
        EXTERN  SYS_WAITING_CYCLES_25

        IMPORT  backup_MSP
        IMPORT  register_backup_table
        IMPORT  register_backup_table_size
        IMPORT  __start_STACK              ; start address for the user stack section. defined in linker script
        IMPORT  __end_STACK                ; end address for the user stack section. defined in linker script
        IMPORT  __start_STACK_GUARD        ; start address for the user stack guard section. defined in linker script
        IMPORT  __end_STACK_GUARD          ; end address for the user stack guard section. defined in linker script
        IMPORT  __start_HEAP               ; start address for the user heap section. defined in linker script
        IMPORT  __end_HEAP                 ; end address for the user heap section. defined in linker script

        EXPORT  CPUcontextSave
        EXPORT  backup_system_register
        EXPORT  restore_system_register

        PUBLIC Reset_Handler
        SECTION .text:CODE:NOROOT:REORDER(2)
Reset_Handler
        LDR     R0, =_bootstack ; Use dedicated stack area to avoid application stack corruption when exiting from Standby state
        MOV     SP, R0          ; set stack pointer
; If we exit from standby mode, restore CPU context and jump to asleep point.
        BL      is_boot_from_standby
        CMP     R0, #1
        BNE     RegularBoot     ; Not an exit from Standby, proceed with a regular reset handling
        BL      StackGuardInit  ; Restore Stack Guard area after it was used as the bootstrap stack
        B       CPUcontextRestore ; Boot from standby, proceed with restoring context
RegularBoot:
        BL      StackGuardInit  ; Restore Stack Guard area after it was used as the bootstrap stack
        LDR     R0, =_estack    ; Load regular stack address
        MOV     SP, R0          ; set stack pointer
; end of specific code section for standby
; Call the clock system initialization function.
        BL      SystemInit

#if STACK_USAGE_DIAGNOSTICS_ENABLED
; Fill entire stack with 0xAA55AA55 watermark
        LDR     R1, = __start_STACK
        LDR     R2, = __end_STACK
        MOV     R3, #0xAA55
        MOVT    R3, #0xAA55
FillStackWatermark:
        CMP     R1, R2
        BGE StackGuardInitCall
        STR     R3, [R1], #4
        B       FillStackWatermark
#endif /* STACK_USAGE_DIAGNOSTICS_ENABLED */

StackGuardInitCall:
        BL      StackGuardInit

HeapWatermarkInit:
#if HEAP_USAGE_DIAGNOSTICS_ENABLED
; Fill user heap area with 0x55AA55AA watermark
        LDR     R1, = __start_HEAP
        LDR     R2, = __end_HEAP
        MOV     R3, #0x55AA
        MOVT    R3, #0x55AA
FillHeapWatermark:
        CMP     R1, R2
        BGE     ProgramStart
        STR     R3, [R1], #4
        B       FillHeapWatermark
#endif /* HEAP_USAGE_DIAGNOSTICS_ENABLED */

ProgramStart:
        LDR     R0, =__iar_program_start
        BX      R0

StackGuardInit:
; Fill stack guard area with 0xDEADBEEF watermark
        LDR     R1, = __start_STACK_GUARD
        LDR     R2, = __end_STACK_GUARD
        MOV     R3, #0xBEEF
        MOVT    R3, #0xDEAD
FillStackGuardWatermark:
        CMP     R1, R2
        IT      GE
        BXGE    LR
        STR     R3, [R1], #4
        B       FillStackGuardWatermark

; These 2 functions are designed to save and then restore CPU context.
CPUcontextSave
        PUSH   { R4 - R12, LR }        ; store R4 to R12 and LR (10 words) onto C stack
        LDR    R4, =backup_MSP         ; load address of backup_MSP into R4
        MOV    R3, SP                  ; load the stack pointer into R3
        STR    R3, [R4]                ; store the MSP into backup_MSP
        DSB
        WFI                            ; all saved, trigger deep sleep

CPUcontextRestore
  ; Even if we fall through the WFI instruction, we will immediately
  ; execute a context restore and end up where we left off with no
  ; ill effects.  Normally at this point the core will either be
  ; powered off or reset (depending on the deep sleep level).
        LDR    R4, =backup_MSP         ; load address of backup_MSP into R4
        LDR    R4, [R4]                ; load the SP from backup_MSP
        MOV    SP, R4                  ; restore the SP from R4
        POP    { R4 - R12, PC }        ; load R4 to R12 and PC (10 words) from C stack

backup_system_register
; R0 -> register_backup_table array current item address
; R1 -> loop counter (from register_backup_table_size to 0)
; R2 -> register_backup_table array current item value

backup_loop_init:
        LDR    R0, =register_backup_table        ; R0 points to the first array item
        LDR    R1, =register_backup_table_size
        LDR    R1, [R1]                          ; R1 contains the number of registers in the array

backup_loop_iter:
        ; Offset processing
        LDR    R2, [R0], #4                      ; R2 contains the register_backup_table current item (register address)
                                                 ; R0 points to the next register_backup_table item (uint32_t items -> 4 bytes added to previous address)

        ; Register value backup
        LDR    R2, [R2]                          ; R2 contains now the register value to backup
        PUSH   {R2}                              ; Push register value into the stack

        ; Loop iteration control
        SUBS   R1, #1                            ; Decrement loop counter by 1 and update APSR (Application Processor Status Register)
        BNE    backup_loop_iter                  ; Loop continues until Z flag is set (still array item to handle)

backup_loop_end:
        BX LR                                    ; Return to caller

restore_system_register
; R0 -> register_backup_table array current item address
; R1 -> loop counter (from register_backup_table_size to 0)
; R2 -> register_backup_table array current item value

restore_loop_init:
        LDR    R0, =register_backup_table         ; R0 points to the first array item

        ; Reverse loop: counter initial value processing
        LDR    R1, =register_backup_table_size
        LDR    R1, [R1]                          ; R1 contains the number of registers in the array
        SUB    R1, #1                            ; R1 now contains last array item index (register_backup_table_size - 1)

        ; Reverse loop: apply offset to current array index (point to last array element)
        ADD    R0, R1, LSL #2                    ; R0 now points to last array item (register_backup_table + (register_backup_table_size - 1) * 4)

        ADD    R1, #1                            ; Re-add 1 to R1 (array length)

; Reverse loop
restore_loop_iter:
        ; Offset processing
        LDR    R2, [R0], #-4                     ; R2 contains the register_backup_table current item (register address)
                                                 ; R0 now points to the previous register_backup_table item (uint32_t items -> 4 bytes subtracted to previous address)

        ; Register value restoration
        POP    {R3}                              ; Head of stack popped into R3. R3 contains register value to restore
        STR    R3, [R2]                          ; Write backuped value into the register

        /* Loop iteration control */
        SUBS   R1, #1                            ; Decrement loop counter by 1 and update APSR (Application Processor Status Register)
        BNE    restore_loop_iter                 ; Loop continues until Z flag is set (still array item to handle)

restore_loop_end:
        BX LR                                    ; Return to caller
; end of specific code section for standby
        END
