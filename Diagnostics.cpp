#include "Synchronized.h"
#include "Diagnostics.h"

Diagnostics::Diagnostics() {
	m_buf = m_buf_a;
	m_buf_len = 0;

	m_log = fopen("/3322-diag.txt", "w");
	m_task = new Task("3322Diagnostics", (FUNCPTR)Diagnostics::InitTask);
	m_flushing = semBCreate(SEM_Q_PRIORITY, SEM_EMPTY);
	m_writing = semBCreate(SEM_Q_PRIORITY, SEM_FULL);

	if (!m_task->Start((INT32)this)) {
		// FIXME: how to use WPI error handling?
	}
	SmartDashboard::init();
}
void Diagnostics::InitTask(Diagnostics *d) {
	d->Run();
}
void Diagnostics::Run() {
	fprintf(m_log, "Diagnostics started\n");
	fflush(m_log);
	while (true) {
		// Wait for a writer to signal ready to flush the buffer
		semTake(m_flushing, WAIT_FOREVER);
		fprintf(m_log, "Data ready\n");
		fflush(m_log);
		char* temp_buf = m_buf;
		int temp_buf_len;
		// Flip buffers and let the writers continue writing
		{
			Synchronized sync(m_writing);
			temp_buf_len = m_buf_len;
			m_buf = (m_buf == m_buf_a) ? m_buf_b : m_buf_a;
			m_buf_len = 0;
		}
		fprintf(m_log, "Buffer flip. len=%d\n", temp_buf_len);
		fflush(m_log);
		// Flush the buffer, then go back to waiting
		fwrite(temp_buf, temp_buf_len, 1, m_log);
		fflush(m_log);
	}
}
void Diagnostics::FlushToDisk() {
	Synchronized sync(m_writing);
	semGive(m_flushing);
}
int Diagnostics::BufferPrintf(const char* format,...)
{
	static unsigned int lastLinePrinted = 0;
	static unsigned int lineNumber = 0;
	Synchronized sync(m_writing);
	lineNumber++;
	if (m_buf_len < DIAG_SIZE - DIAG_LINE_SIZE) {
		// The writer must guarantee never to use more than DIAG_LINE_SIZE bytes.
		// If that amount of buffer space is not available, it means buffer flushing
		// is lagging behind. Skip writing until the buffer flushes. TODO: it's
		// not clear whether new snapshots are more valuable than old -- we could
		// push older snapshots out of the buffer if the buffer is full.
		// ***Posible solution could be useing circular buffers
		if (lastLinePrinted + 1 != lineNumber) {
			m_buf[m_buf_len++] = '*';
			m_buf[m_buf_len++] = '\n';
		}
	    va_list args;
	    va_start(args, format);
	    //now print actual message
	    int len = vsprintf(m_buf + m_buf_len, format, args);
	    //vsprintf returns -1 if it fails. This is to prevent m_buf_len from being decremented
	    if (len > 0) m_buf_len += len;
	    va_end(args);
	    lastLinePrinted = lineNumber;
	    if (m_buf_len > DIAG_SIZE / 2) {
			// If the buffer is full enough, start flushing.
			semGive(m_flushing);
		}
	    return len;
	}
	return -1;
}
int Diagnostics::DashPrintf(const char* key, const char* format, ...) {
	if(!key || !format)return -1;
    va_list args;
    va_start(args, format);
    char buff[256];
    int len = vsnprintf(buff,256, format, args);
    SmartDashboard::PutString(key,buff);
    va_end(args);
    return len;
}
