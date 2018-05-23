#ifndef COMMON_H_INCLUDED
#define COMMON_H_INCLUDED


#define LO8(x) (uint8_t)( x & 0x00FF )
#define HI8(x) (uint8_t)( (x & 0xFF00 ) >> 8 )
#define LO16(x) (uint16_t)( x & 0x0000FFFF )
#define HI16(x) (uint16_t)( (x & 0xFFFF0000 ) >> 16 )

#define __enter_critical() {uint32_t irq; irq = __get_PRIMASK();
#define __exit_critical() __set_PRIMASK(irq);}
#define ATOMIC_SECTION(X) __enter_critical(); {X}; __exit_critical();




#endif /* COMMON_H_INCLUDED */
