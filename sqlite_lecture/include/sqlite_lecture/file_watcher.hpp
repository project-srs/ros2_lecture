#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <sys/inotify.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <string>

#define EVENT_SIZE (sizeof(struct inotify_event))
#define BUF_LEN (1024 * (EVENT_SIZE + 16))

class FileWatcher
{
public:
  FileWatcher(void)
  {
  }

  ~FileWatcher()
  {
    if (0 <= fd_ && 0 <= wd_) {
      inotify_rm_watch(fd_, wd_);
      close(fd_);
    }
  }

  bool setFilePath(const std::string & file_path)
  {
    fd_ = inotify_init();
    if (fd_ == -1) {
      perror("inotify_init");
      return false;
    }
    fcntl(fd_, F_SETFL, fcntl(fd_, F_GETFL) | O_NONBLOCK);

    wd_ = inotify_add_watch(fd_, file_path.c_str(), IN_MODIFY | IN_CREATE | IN_DELETE);
    return true;
  }

  bool checkIfFileModified(void)
  {
    int count = 0;
    int i = 0;
    char buffer[BUF_LEN];
    int length = read(fd_, buffer, BUF_LEN);

    while (i < length) {
      struct inotify_event * event = (struct inotify_event *)&buffer[i];
      if (event->mask & IN_MODIFY) {
        count++;
      }
      i += EVENT_SIZE + event->len;
    }
    return 0 < count;
  }

private:
  int fd_{-1};
  int wd_{-1};
};
