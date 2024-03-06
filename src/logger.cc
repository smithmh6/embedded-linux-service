/* Copyright 2023 Thorlabs, Inc
 */


#include <ctime>
#include <cstring>
#include <fstream>
#include <iostream>
#include <dow_gen3_embedded/inc/logger.h>

Logger::Logger() :
  logdir_(NULL), logname_(NULL), loglevel_(Info), max_file_size_(0) {
  /* intentionally empy */
}

Logger::Logger(const char* logdir,
               const char* logname,
               LogLevel level,
               double max_file_size) {
  // set private class members
  logdir_ = logdir;
  logname_ = logname;
  loglevel_ = level;
  max_file_size_ = max_file_size;

  // generate the file path
  time_t now;
  time(&now);
  char tbuf[sizeof("20111008T_070709Z")];
  strftime(tbuf, sizeof(tbuf), "%Y%m%dT%H%M%SZ", gmtime(&now));

  asprintf(&logpath_, "%s/%s_%s.log", logdir_, tbuf, logname_);

  if (loglevel_ <= Debug) {
    char *msgbuf;
    asprintf(&msgbuf, "(%s) logging to path %s", logname_, logpath_);
    Log(Debug, msgbuf);
    free(msgbuf);
  }

  if (loglevel_ <= Info) {
    time(&now);
    char tbuf2[sizeof("2011-10-08T07:07:09Z")];
    strftime(tbuf2, sizeof(tbuf2), "%FT%TZ", gmtime(&now));

    char *msgbuf;
    asprintf(&msgbuf, "Logging started at %s", tbuf2);
    Log(Info, msgbuf);
    free(msgbuf);
  }
}

void Logger::SetLogLevel(LogLevel level) {
  if (level != loglevel_) {
    loglevel_ = level;
  }
}

void Logger::RotateLogFile() {
  // generate a new timestamp
  time_t now;
  time(&now);
  char tbuf[sizeof("20111008T_070709Z")];
  strftime(tbuf, sizeof(tbuf), "%Y%m%dT%H%M%SZ", gmtime(&now));

  // free logpath and reset
  free(logpath_);
  asprintf(&logpath_, "%s/%s_%s.log", logdir_, tbuf, logname_);

  // reset bytes_written
  bytes_written_ = 0;

  if (loglevel_ <= Debug) {
    char *msgbuf;
    asprintf(&msgbuf, "(%s) file size limit reached, now logging to path %s", logname_, logpath_);
    Log(Debug, msgbuf);
    free(msgbuf);
  }
}

void Logger::Log(LogLevel level, const char* message) {
  if (level >= loglevel_) {
    // get iso timestamp
    time_t now;
    time(&now);
    char buf[sizeof("2011-10-08T07:07:09Z")];
    strftime(buf, sizeof(buf), "%FT%TZ", gmtime(&now));

    std::ofstream FILE(logpath_, std::ios_base::app);
    FILE << "[" << buf << "]\t";  // log the timestamp

    size_t n_bytes = strlen(message);

    // switch on log level, output to file
    switch (level) {
      case Trace:
        FILE << "[Trace]\t";
        std::cout << "[Trace]\t" << message << "  " << n_bytes << std::endl;
        break;

      case Debug:
        FILE << "[Debug]\t";
        std::cout << "[Debug]\t" << message << "  " << n_bytes << std::endl;
        break;

      case Info:
        FILE << "[Info]\t";
        break;

      case Warn:
        FILE << "[Warn]\t";
        break;

      case Error:
        FILE << "[Error]\t";
        break;

      case Fatal:
        FILE << "[Fatal]\t";
        break;
    }

    // write message and close file object
    FILE << message << "  " << n_bytes << "\n";
    FILE.close();

    // check if bytes written >= max bytes (in MB, 1MB = 1e6 btyes)
    // pad with 100 bytes so it doesn't exceed the limit (100b - 1e-4 MB)
    bytes_written_ += (double)n_bytes;
    if (bytes_written_ >= (max_file_size_ - 0.0001) * 1e6) {
      RotateLogFile();
    }
  }
}
