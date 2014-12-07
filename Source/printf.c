/*
  File: printf.c

  Copyright (C) 2004,2008  Kustaa Nyholm

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

*/

#include <stdint.h>

#ifdef ITM_TRACE
#if defined (CORTEX_M4) && defined (STM32F4xx)
#include "stm32f4xx.h"
#elif defined (CORTEX_M4) && defined (STM32F30x)
#include "stm32f30x.h"
#elif defined (CORTEX_M3) && defined (STM32F10x)
#include "stm32f10x.h"
#else
#error No core defined!
#endif
#elif MEM_TRACE
#include "memtrace.h"
#else
#include "serial.h"
#endif

#include "printf.h"

static char* bf;
static char buf[12];
static unsigned int num;
static char uc;
static char zs;

static void out(char c) {
  *bf++ = c;
}

static void outDgt(char dgt) {
  out(dgt+(dgt<10 ? '0' : (uc ? 'A' : 'a')-10));
  zs=1;
}

static void divOut(unsigned int div) {
  unsigned char dgt=0;
  num &= 0xffffffff;
  while (num>=div) {
    num -= div;
    dgt++;
  }
  if (zs || dgt>0)
    outDgt(dgt);
}


int  t_puts(const char *s)
{
  char *p;

  p = (char *) s;
  while (*p != '\0')
    {
#ifdef ITM_TRACE
      ITM_SendChar ((uint32_t) *p);
#elif MEM_TRACE
      mem_putchar(*p);
#else
      putchar(*p);
#endif
      p++;
    }
#if defined (TFP_PUTS_ENDL)
#ifdef ITM_TRACE
  ITM_SendChar ((uint32_t) 10);
#elif MEM_TRACE
  mem_putchar(10);
#else
  putchar(10);
#endif
#endif
  return 1;
}

void t_printf(const char *fmt, ...)
{
  va_list va;
  char ch;
  char* p;

  va_start(va,fmt);

  while ((ch=*(fmt++))) {
    if (ch!='%') {
#ifdef ITM_TRACE
      ITM_SendChar ((uint32_t) ch);
#elif MEM_TRACE
  mem_putchar(ch);
#else
      putchar(ch);
#endif
    }
    else {
      char lz=0;
      char w=0;
      ch=*(fmt++);
      if (ch=='0') {
	ch=*(fmt++);
	lz=1;
      }
      if (ch>='0' && ch<='9') {
	w=0;
	while (ch>='0' && ch<='9') {
	  w=(((w<<2)+w)<<1)+ch-'0';
	  ch=*fmt++;
	}
      }
      bf=buf;
      p=bf;
      zs=0;
      switch (ch) {
      case 0:
	goto abort;
      case 'u':
      case 'd' :
	num=va_arg(va, unsigned int);
	if (ch=='d' && (int)num<0) {
	  num = -(int)num;
	  out('-');
	}
	divOut(1000000000);
	divOut(100000000);
	divOut(10000000);
	divOut(1000000);
	divOut(100000);
	divOut(10000);
	divOut(1000);
	divOut(100);
	divOut(10);
	outDgt(num);
	break;
      case 'x':
      case 'X' :
	uc= ch=='X';
	num=va_arg(va, unsigned int);
	divOut(0x10000000);
	divOut(0x1000000);
	divOut(0x100000);
	divOut(0x10000);
	divOut(0x1000);
	divOut(0x100);
	divOut(0x10);
	outDgt(num);
	break;
      case 'c' :
	out((char)(va_arg(va, int)));
	break;
      case 's' :
	p=va_arg(va, char*);
	break;
      case '%' :
	out('%');
      default:
	break;
      }
      *bf=0;
      bf=p;
      while (*bf++ && w > 0)
	w--;
      while (w-- > 0)
	{
#ifdef ITM_TRACE
	  ITM_SendChar ((uint32_t) (lz ? '0' : ' '));
#elif MEM_TRACE
	  mem_putchar(lz ? '0' : ' ');
#else
	  putchar(lz ? '0' : ' ');
#endif
	}

      while ((ch= *p++))
	{
#ifdef ITM_TRACE
	  ITM_SendChar ((uint32_t) ch);
#elif MEM_TRACE
	  mem_putchar(ch);
#else
	  putchar(ch);
#endif
	}
    }
  }
 abort:;
  va_end(va);
}

void t_snprintf(char *str, int size, const char *fmt, ...)
{ 
  va_list va;
  char ch;
  char* p;
  char *outp;
  int32_t nout;

  outp = str;
  nout = 0;

  va_start(va,fmt);

  while ((ch=*(fmt++))) 
    {
      if (ch!='%') 
	{
	  if (nout < size)
	    {
	      *(outp++) = ch;
	      nout++;
	    }
	}
      else {
	char lz=0;
	char w=0;
	ch=*(fmt++);
	if (ch=='0') {
	  ch=*(fmt++);
	  lz=1;
	}
	if (ch>='0' && ch<='9') {
	  w=0;
	  while (ch>='0' && ch<='9') {
	    w=(((w<<2)+w)<<1)+ch-'0';
	    ch=*fmt++;
	  }
	}
	bf=buf;
	p=bf;
	zs=0;
	switch (ch) {
	case 0:
	  goto abort;
	case 'u':
	case 'd' :
	  num=va_arg(va, unsigned int);
	  if (ch=='d' && (int)num<0) {
	    num = -(int)num;
	    out('-');
	  }
	  divOut(10000);
	  divOut(1000);
	  divOut(100);
	  divOut(10);
	  outDgt(num);
	  break;
	case 'x':
	case 'X' :
	  uc= ch=='X';
	  num=va_arg(va, unsigned int);
	  divOut(0x1000);
	  divOut(0x100);
	  divOut(0x10);
	  outDgt(num);
	  break;
	case 'c' :
	  out((char)(va_arg(va, int)));
	  break;
	case 's' :
	  p=va_arg(va, char*);
	  break;
	case '%' :
	  out('%');
	default:
	  break;
	}
	*bf=0;
	bf=p;
	while (*bf++ && w > 0)
	  w--;
	while (w-- > 0)
	  {
	    if (nout < size)
	      {
		*(outp++) = lz ? '0' : ' ';
		nout++;
	      }

	  }
	while ((ch= *p++))
	  {
	    if (nout < size)
	      {
		*(outp++) = ch;
		nout++;
	      }
	  
	  }
      }
    }
 abort:
  str[nout] = '\0';
  va_end(va);
}
