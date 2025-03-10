ENTRY(entry)

MEMORY {
    rom (rx) : ORIGIN = 0x00000000, LENGTH = 0x2000 /* Ajusta la longitud según tus necesidades */
}

SECTIONS {
    .text 0x0 : {
        *(.text)
        . = ALIGN(4); /* Alinea la posición actual a un múltiplo de 4 bytes */
        FILL(0x00000013)
        . += 0x50 - .;
        *(.debug_entry)
        . = ALIGN(4); /* Alinea la posición actual a un múltiplo de 4 bytes */
        FILL(0x00000013)
        . += 0x200 - .;        
        *(.shadow_wfi)
    } > rom

}