#include "main.h"
#include "math.h"

u8 back0time(int position)
{
	u16 time;
	u32 pos = fabs(position);
	
	if(pos <= 4096)
		time = 10;
	else if(pos > 4096)
		time  = 10 * ( (pos-4096)/4096/2+1 );
	
	return time;
}

