/*
Common part of the linker scripts for STR71x devices in FLASH mode
(that is, the FLASH is seen at 0)
Copyright RAISONANCE 2005
You can use, modify and distribute thisfile freely, but without any waranty.
*/



/* Sections Definitions */

SECTIONS
{
    /* for Cortex devices, the beginning of the startup code is stored in the .isr_vector section, which goes to FLASH */
    .isr_vector :
    {
	. = ALIGN(4);
        KEEP(*(.isr_vector))            /* Startup code */
	. = ALIGN(4);
    } >FLASH

    /* for some STRx devices, the beginning of the startup code is stored in the .flashtext section, which goes to FLASH */
    .flashtext :
    {
	. = ALIGN(4);
        *(.flashtext)            /* Startup code */
	. = ALIGN(4);
    } >FLASH


    /* the program code is stored in the .text section, which goes to Flash */
    .text :
    {
	    . = ALIGN(4);
      _stext = .;

        *(.text)                   /* remaining code */
        *(.text.*)                 /* remaining code */
        *(.rodata)                 /* read-only data (constants) */
        *(.rodata*)
        *(.glue_7)
        *(.glue_7t)

        /* Some stuff of CodeSourcery? */
        *(.init)
        *(.fini)
        *(.eh_frame)
        
        /* Parameters */
        . = ALIGN(4);
        _param_start = .;
        KEEP(*(.param))
        KEEP(*(.param.*))
        _param_stop = .;
        /* Logging */
        . = ALIGN(4);
        _log_start = .;
        KEEP(*(.log))
        KEEP(*(.log.*))
        _log_stop = .;
        /* Decks */
	    . = ALIGN(4);
        _deckDriver_start = .;
        KEEP(*(.deckDriver));
        KEEP(*(.deckDriver.*));
        _deckDriver_stop = .;
        /* Event Triggers */
	    . = ALIGN(4);
        _eventtrigger_start = .;
        KEEP(*(.eventtrigger));
        KEEP(*(.eventtrigger.*));
        _eventtrigger_stop = .;

   	 _etext = .;
    } >FLASH

    .libc :
    {
      . = ALIGN(4);
      *(.ARM.exidx*)
      . = ALIGN(4);

      _elibc = .;
    } >FLASH

    /* for some LPC devices, there is a FLASH patch to place at a specified address */
    .flashpatch :
    {
	. = ALIGN(4);
        KEEP(*(.flashpatch))            /* flashpatch data */
	. = ALIGN(4);
    } >FLASHPATCH

    /* for some LPC devices, there is a FLASH patch to place at a specified address
    and then there is the rest of the flash */
    .endflash :
    {
	. = ALIGN(4);
        *(.endflash)            /* endflash code */
	. = ALIGN(4);
    } >ENDFLASH

    /* This is the initialized data section
    The program executes knowing that the data is in the RAM
    but the loader puts the initial values in the FLASH (inidata).
    It is one task of the startup to copy the initial values from FLASH to RAM. */

    .data  :
    {
	    . = ALIGN(4);
        /* This is used by the startup in order to initialize the .data secion */
        _sdata = . ;

        *(.data)
        *(.data.*)
        *(.RAMtext)

	    . = ALIGN(4);
	    /* This is used by the startup in order to initialize the .data secion */
   	 _edata = . ;
    } >RAM AT> FLASH

    /* The address where initialization data is stored in flash, used by the start up to copy init data from flash to RAM */
    _sidata = LOADADDR(.data);


    /* This is the uninitialized data section. This memory is zeroed at start up */
    .bss :
    {
	    . = ALIGN(4);
        /* This is used by the startup in order to initialize the .bss secion */
        _sbss = .;

        *(.bss)
        *(.bss.*)
        *(COMMON)

	    . = ALIGN(4);
	    /* This is used by the startup in order to initialize the .bss secion */
   	 _ebss = . ;
    } >RAM


    /* This is the uninitialized data section for the CCM RAM. This memory is zeroed at start up */
    .ccmbss (NOLOAD) :
    {
	    . = ALIGN(4);
        /* This is used by the startup in order to initialize the .bss secion */
        _sccmbss = .;
        *(.ccmbss)
        *(.ccmbss*)

	    . = ALIGN(4);
	    /* This is used by the startup in order to initialize the .bss secion */
   	 _eccmbss = .;
    } >CCMRAM

    /* This is the initialized data section for the CCM RAM
    The program executes knowing that the data is in the CCM RAM
    but the loader puts the initial values in the FLASH.
    It is one task of the startup to copy the initial values from FLASH to RAM. */
    .ccmdata :
    {
        . = ALIGN(4);
        _sccmdata = .;        /* create a global symbol at data start */
        *(.ccmdata)
        *(.ccmdata*)

        . = ALIGN(4);
        _eccmdata = .;        /* define a global symbol at data end */
    } >CCMRAM AT> FLASH

    /* The address where initialization data is stored in flash, used by the start up to copy init data from flash to RAM */
    _siccmdata = LOADADDR(.ccmdata);


    /* This is uninitialized data that is NOT zeroed at start up. */
    .nzds :
    {
	    . = ALIGN(4);
        _snzds = .;

        *(.nzds)
        *(.nzds.*)
        *(COMMON)

	    . = ALIGN(4);
   	 _enzds = . ;
    } >RAM


    PROVIDE ( end = _enzds );
    PROVIDE ( _end = _enzds );


    /* This is the user stack section
    This is just to check that there is enough RAM left for the User mode stack
    It should generate an error if it's full.
     */
    ._usrstack :
    {
	    . = ALIGN(4);
        _susrstack = . ;

        . = . + _Minimum_Stack_Size ;

	    . = ALIGN(4);
        _eusrstack = . ;
    } >RAM



    /* this is the FLASH Bank1 */
    /* the C or assembly source must explicitly place the code or data there
    using the "section" attribute */
    .b1text :
    {
        *(.b1text)                   /* remaining code */
        *(.b1rodata)                 /* read-only data (constants) */
        *(.b1rodata*)
    } >FLASHB1

    /* this is the EXTMEM */
    /* the C or assembly source must explicitly place the code or data there
    using the "section" attribute */

    /* EXTMEM Bank0 */
    .eb0text :
    {
        *(.eb0text)                   /* remaining code */
        *(.eb0rodata)                 /* read-only data (constants) */
        *(.eb0rodata*)
    } >EXTMEMB0

    /* EXTMEM Bank1 */
    .eb1text :
    {
        *(.eb1text)                   /* remaining code */
        *(.eb1rodata)                 /* read-only data (constants) */
        *(.eb1rodata*)
    } >EXTMEMB1

    /* EXTMEM Bank2 */
    .eb2text :
    {
        *(.eb2text)                   /* remaining code */
        *(.eb2rodata)                 /* read-only data (constants) */
        *(.eb2rodata*)
    } >EXTMEMB2

    /* EXTMEM Bank0 */
    .eb3text :
    {
        *(.eb3text)                   /* remaining code */
        *(.eb3rodata)                 /* read-only data (constants) */
        *(.eb3rodata*)
    } >EXTMEMB3

    __exidx_start = .;
    __exidx_end = .;

    /* after that it's only debugging information. */

    /* remove the debugging information from the standard libraries */
    /DISCARD/ :
    {
     libc.a ( * )
     libm.a ( * )
     libgcc.a ( * )
     }

    /* Stabs debugging sections.  */
    .stab          0 : { *(.stab) }
    .stabstr       0 : { *(.stabstr) }
    .stab.excl     0 : { *(.stab.excl) }
    .stab.exclstr  0 : { *(.stab.exclstr) }
    .stab.index    0 : { *(.stab.index) }
    .stab.indexstr 0 : { *(.stab.indexstr) }
    .comment       0 : { *(.comment) }
    /* DWARF debug sections.
       Symbols in the DWARF debugging sections are relative to the beginning
       of the section so we begin them at 0.  */
    /* DWARF 1 */
    .debug          0 : { *(.debug) }
    .line           0 : { *(.line) }
    /* GNU DWARF 1 extensions */
    .debug_srcinfo  0 : { *(.debug_srcinfo) }
    .debug_sfnames  0 : { *(.debug_sfnames) }
    /* DWARF 1.1 and DWARF 2 */
    .debug_aranges  0 : { *(.debug_aranges) }
    .debug_pubnames 0 : { *(.debug_pubnames) }
    /* DWARF 2 */
    .debug_info     0 : { *(.debug_info .gnu.linkonce.wi.*) }
    .debug_abbrev   0 : { *(.debug_abbrev) }
    .debug_line     0 : { *(.debug_line) }
    .debug_frame    0 : { *(.debug_frame) }
    .debug_str      0 : { *(.debug_str) }
    .debug_loc      0 : { *(.debug_loc) }
    .debug_macinfo  0 : { *(.debug_macinfo) }
    /* SGI/MIPS DWARF 2 extensions */
    .debug_weaknames 0 : { *(.debug_weaknames) }
    .debug_funcnames 0 : { *(.debug_funcnames) }
    .debug_typenames 0 : { *(.debug_typenames) }
    .debug_varnames  0 : { *(.debug_varnames) }
}
