#include "IA32.H"

#define GENREGS 8
#define SEGREGS 6
#define SPRREGS 2
#define INVALID_REG 0xFFFFFFFF

int IA32::nRegs(void)
{
	return GENREGS+SEGREGS+SPRREGS;
}

ac_word IA32::reg_read ( int reg )
{
	if ( (reg >= 0) && (reg < GENREGS) )
		return GR.read(reg);
	else if ( (reg >= GENREGS) && (reg < (GENREGS+SPRREGS)) )
		return SPR.read(1-(reg-(GENREGS)));
	else if ( (reg >= (GENREGS+SPRREGS)) && (reg < (GENREGS+SEGREGS+SPRREGS)) )
		return SR.read(reg-(GENREGS+SPRREGS));
	return INVALID_REG;
}

void IA32::reg_write( int reg, ac_word value )
{
	if ( (reg >= 0) && (reg < GENREGS) )
		GR.write(reg, value);
	else if ( (reg >= GENREGS) && (reg < (GENREGS+SPRREGS)) )
		SPR.write((1-(reg-GENREGS)), value);
	else if ( (reg >= (GENREGS+SPRREGS)) && (reg < (GENREGS+SEGREGS+SPRREGS)) )
		SR.write((reg-(GENREGS+SPRREGS)), value);
}

unsigned char IA32::mem_read ( unsigned int address )
{
	return ac_resources::IM->read_byte(address);
}

void IA32::mem_write ( unsigned int address, unsigned char byte )
{
	ac_resources::IM->write_byte(address,byte);
}

