#include <mach/platform.h>

#define pilot_REVISION_1_0 1
#define pilot_REVISION_1_1 2
#define pilot_REVISION_1_2 3

/* gpio configurations */
/* rpi gpios used depend of the rpi revision and the pilot revision */
#define RPI_REVISION 2
#define pilot_REVISION pilot_REVISION_1_2

#if pilot_REVISION == pilot_REVISION_1_0
#define DATA_M2R 17
#elif pilot_REVISION == pilot_REVISION_1_1
#define DATA_M2R 8
#elif pilot_REVISION == pilot_REVISION_1_2
#define DATA_M2R 27
#else
compile error - undefined revision
#endif

#ifndef BCM2708_PERI_BASE
  #define BCM2708_PERI_BASE 0x20000000
  #define GPIO_BASE (BCM2708_PERI_BASE + 0x200000)
  #define SPI0_BASE (BCM2708_PERI_BASE + 0x204000)
#endif

//********************************************************************
// START GPIO specific defines and functions
#define GPIO_BASE (BCM2708_PERI_BASE + 0x200000)

/* sets bits which are 1 ignores bits which are 0 */
#define GPIO_SETALL(o) (*(_internals.GpioSet)=o)

/* sets the specified output */
#define GPIO_SET(o) (GPIO_SETALL(1 << (o)))

/* gets the specified input */
#define GPIO_GET(i) (*(_internals.GpioGet) >> (i) & 1)

/* clears bits which are 1 ignores bits which are 0 */
#define GPIO_CLRALL *(_internals.GpioClr) 

/* clears the specified output */
#define GPIO_CLR(o) (GPIO_CLRALL = 1 << (o))

// #define GPIO_SETALL(gpio) *(gpio+7)  // sets bits which are 1 ignores bits which are 0
// #define GPIO_SET(gpio, o) (GPIO_SETALL(gpio) = 1 << (o)) // sets the specified output
// #define GPIO_GET(gpio,i) (*(gpio) >> (i) & 1)

// ****************************************************************************
// SPI defines 
#define SPI0_CNTLSTAT(spi) *(spi + 0)
#define SPI0_FIFO(spi) *(spi + 1)
#define SPI0_CLKSPEED(spi) *(spi + 2)


#define SPI0_CS_DONE     0x00010000 /* DONE - Transfer done */  // SPI transfer done. WRT to CLR!

#define SPI0_CS_RXD (1 << 17) /* RXD RX FIFO contains DATA. 0 = RX FIFO is empty. 1 = RX FIFO contains at least 1 byte. */

#define SPI0_CS_TXFIFOSPCE   0x00040000 // Transmit FIFO has space
#define SPI0_CS_RXFIFODATA   0x00020000 // Receive FIFO has data

#define SPI0_CS_ACTIVATE 0x00000080 /* TA - Transfer Active */ // Activate: be high before starting 
#define SPI0_CS_CLRFIFOS 0x00000030
#define SPI0_CS_CHIPSEL0 0x00000000
#define SPI0_CS_CHIPSEL1 0x00000001
#define SPI0_CS_CHIPSEL2 0x00000002
#define SPI0_CS_CHIPSELN 0x00000003
#define SPI0_CS_CPHA     0x00000004

#define SPI0_CS_CLRALL (SPI0_CS_CLRFIFOS | SPI0_CS_DONE)
// ****************************************************************************

