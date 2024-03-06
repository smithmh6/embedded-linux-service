/* Copyright 2023 Thorlabs, Inc.
 * Heath Smith
 * 11/8/2023
 *
 */

#ifndef INC_LOGGER_H_
#define INC_LOGGER_H_

enum LogLevel {
    Trace, Debug, Info, Warn, Error, Fatal
};

class Logger {
 public:
  Logger();

  Logger(const char* logdir,
         const char* logname,
         LogLevel level,
         double max_file_size);

  inline const char* logdir() const { return logdir_; }

  inline const char* logname() const { return logname_; }

  inline char* logpath() const { return logpath_; }

  inline LogLevel loglevel() const { return loglevel_; }

  inline double max_file_size() const { return max_file_size_; }

  inline double bytes_written() const { return bytes_written_; }

  void SetLogLevel(LogLevel level);

  void RotateLogFile();

  void Log(LogLevel level, const char* message);

 private:
  const char* logdir_;
  const char* logname_;
  char* logpath_;
  LogLevel loglevel_;
  double max_file_size_;
  double bytes_written_;
};

#endif  // INC_LOGGER_H_
