#ifndef DUMMYLOGGER_H_
#define DUMMYLOGGER_H_

//#define LOGGER_ENABLED

#if defined(LOGGER_ENABLED)
#include <logger.h>

#define DlogPrintA(text) logPrintA(text)
#define DlogPrintE(text) logPrintE(text)
#define DlogPrintV(text) logPrintV(text)
#define DlogPrintD(text) logPrintD(text)
#define DlogPrintI(text) logPrintI(text)
#define DlogPrintW(text) logPrintW(text)
#define DlogPrintlnA(text) logPrintlnA(text)
#define DlogPrintlnE(text) logPrintlnE(text)
#define DlogPrintlnV(text) logPrintlnV(text)
#define DlogPrintlnD(text) logPrintlnD(text)
#define DlogPrintlnI(text) logPrintlnI(text)
#define DlogPrintlnW(text) logPrintlnW(text)

#else

#define DlogPrintA(text) do {} while(0)
#define DlogPrintE(text) do {} while(0)
#define DlogPrintV(text) do {} while(0)
#define DlogPrintD(text) do {} while(0)
#define DlogPrintI(text) do {} while(0)
#define DlogPrintW(text) do {} while(0)
#define DlogPrintlnA(text) do {} while(0)
#define DlogPrintlnE(text) do {} while(0)
#define DlogPrintlnV(text) do {} while(0)
#define DlogPrintlnD(text) do {} while(0)
#define DlogPrintlnI(text) do {} while(0)
#define DlogPrintlnW(text) do {} while(0)

#endif

#endif
