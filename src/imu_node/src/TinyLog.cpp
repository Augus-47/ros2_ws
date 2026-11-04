
#include "imu_node/TinyLog.h"

#include <stdarg.h>
#include <thread>
#include <mutex>
#include <string>
#include <string.h>
#include <atomic>
#include <sstream>

#ifdef _WIN32
#include <direct.h>
#else
#include <unistd.h>
#include <sys/stat.h>
#endif

int TinyLog::storageLevel = TinyLog::INFO; /* storage to file , default INFO */

int TinyLog::singleMaxSize = 10485760; /* unit��Byte��default 10MB */

std::string TinyLog::storageDir = "./"; /* storage to dir , default ./log */

int TinyLog::logMode = TinyLog::MULTI_THREAD;

void TinyLog::debug(const char* format, ...) 
{
	va_list args;
	va_start(args, format);

	logConstruct(TinyLog::DEBUG, format, args);

	va_end(args);
}

void TinyLog::info(const char* format, ...)
{
	va_list args;
	va_start(args, format);

	logConstruct(TinyLog::INFO, format, args);
	
	va_end(args);
}

void TinyLog::warning(const char* format, ...)
{
	va_list args;
	va_start(args, format);

	logConstruct(TinyLog::WARNING, format, args);

	va_end(args);
}

void TinyLog::error(const char* format, ...)
{
	va_list args;
	va_start(args, format);

	logConstruct(TinyLog::ERROR, format, args);

	va_end(args);
}

void TinyLog::fatal(const char* format, ...)
{
	va_list args;
	va_start(args, format);

	logConstruct(TinyLog::FATAL, format, args);

	va_end(args);
}

void TinyLog::setStorageLevel(int level)
{
	switch (level)
	{
	case TinyLog::DEBUG:
		storageLevel = TinyLog::DEBUG;
		break;
	case TinyLog::INFO:
		storageLevel = TinyLog::INFO;
		break;
	case TinyLog::WARNING:
		storageLevel = TinyLog::WARNING;
		break;
	case TinyLog::ERROR:
		storageLevel = TinyLog::ERROR;
		break;
	case TinyLog::FATAL:
		storageLevel = TinyLog::FATAL;
		break;
	default:
		break;
	}
}

void TinyLog::setSingleMaxSize(int size)
{
	singleMaxSize = size;
}

void TinyLog::setStorageDir(const char* dir)
{
	storageDir = dir;
}

void TinyLog::setLogMode(int mode)
{
	switch (mode)
	{
	case TinyLog::SINGLE_THREAD:
		logMode = TinyLog::SINGLE_THREAD;
		break;
	case TinyLog::MULTI_THREAD:
		logMode = TinyLog::MULTI_THREAD;
		break;
	default:
		break;
	}
}

int TinyLog::fileSize(const char *path)
{
	int size = -1;
	FILE *fp = fopen(path, "r");
	if (fp)
	{
		fseek(fp, 0L, SEEK_END);
		size = ftell(fp);
		fclose(fp);
	}
	return size;
}

void TinyLog::logConstruct(const int& level,
	const char* format,
	va_list args)
{
	switch (logMode)
	{
	case TinyLog::SINGLE_THREAD:
		break;
	case TinyLog::MULTI_THREAD:
		multiThreadConstruct(level, format, args);
		break;
	default:
		break;
	}
}


void TinyLog::multiThreadConstruct(const int& level,
	const char* format,
	va_list args)
{
	const char* logLevel = NULL;
	switch (level)
	{
	case TinyLog::DEBUG:
		logLevel = "DEBUG";
		break;
	case TinyLog::INFO:
		logLevel = "INFO";
		break;
	case TinyLog::WARNING:
		logLevel = "WARNING";
		break;
	case TinyLog::ERROR:
		logLevel = "ERROR";
		break;
	case TinyLog::FATAL:
		logLevel = "FATAL";
		break;
	default:
		logLevel = "";
		break;
	}

	time_t tt = time(NULL);
	tm* t = localtime(&tt);

	char content[2048] = { 0 };
	char log[sizeof(content) + 50] = { 0 };

	std::ostringstream oss;
	oss << std::this_thread::get_id();
	std::string threadId = oss.str();

	vsprintf(content, format, args);
	sprintf(log,
		"[%s] [%d-%02d-%02d %02d:%02d:%02d] %s\n",
		logLevel,
		t->tm_year + 1900,
		t->tm_mon + 1,
		t->tm_mday,
		t->tm_hour,
		t->tm_min,
		t->tm_sec,
		/*threadId.c_str(),*/
		content);
	printf("%s", log);

	if (level >= storageLevel)
	{
		std::string filename = storageDir;
		if (fileSize(filename.c_str()) > singleMaxSize)
		{
			std::chrono::milliseconds now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
			long long msTimestamp = now.count();
			std::string backupFilename = filename + "." + std::to_string(msTimestamp);
			rename(filename.c_str(), backupFilename.c_str());
		}
		else
		{
			FILE *fp = NULL;
			fp = fopen(filename.c_str(), "a+");
			if (fp != NULL)
			{
				fputs(log, fp);
				fclose(fp);
				fp = NULL;
			}
			else
			{
				static bool printWarning = false;
				if(false == printWarning)
				{
					printWarning = true;
					printf("[TinyLog::logConstruct]: open or create log file failed!\n");
				}
			}
		}
	}
}