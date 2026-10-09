#include <fcntl.h>
#include <gtest/gtest.h>
#include <stdlib.h>
#include <unistd.h>

#include <string>

#include "mg_drivers/base/serial_communicator2.hpp"

using mg_drivers::SerialCommunicator2;

namespace
{
// 疑似端末を作り、スレーブ側のパスを返す。マスター側は閉じずに保持する (閉じるとスレーブの読み書きが失敗する)
class PseudoTerminal
{
public:
  PseudoTerminal()
  {
    master_fd_ = posix_openpt(O_RDWR | O_NOCTTY);
    grantpt(master_fd_);
    unlockpt(master_fd_);
    slave_path_ = ptsname(master_fd_);
  }
  ~PseudoTerminal() { close(master_fd_); }

  const std::string & slave_path() const { return slave_path_; }

private:
  int master_fd_;
  std::string slave_path_;
};
}  // namespace

TEST(SerialCommunicator2, ReadWithoutDataReturnsEmpty)
{
  PseudoTerminal pty;
  SerialCommunicator2 serial(pty.slave_path());
  ASSERT_TRUE(serial.is_open_serial_);

  // 非ブロッキングの read は、データが無いと -1 を返す。例外にせず空を返すこと
  EXPECT_TRUE(serial.serial_read(1000).empty());
}

TEST(SerialCommunicator2, ResetMissingDeviceLeavesPortClosed)
{
  SerialCommunicator2 serial("/dev/ttyRobot-not-exist");
  EXPECT_FALSE(serial.is_open_serial_);
  EXPECT_FALSE(serial.is_device_present());

  serial.reset_serial();
  EXPECT_FALSE(serial.is_open_serial_);
  EXPECT_TRUE(serial.serial_read(1000).empty());
  EXPECT_LE(serial.serial_write({0x01}), 0);
}

TEST(SerialCommunicator2, CloseAndResetReopensPort)
{
  PseudoTerminal pty;
  SerialCommunicator2 serial(pty.slave_path());
  ASSERT_TRUE(serial.is_open_serial_);
  EXPECT_TRUE(serial.is_device_present());

  serial.close_serial();
  EXPECT_FALSE(serial.is_open_serial_);

  serial.reset_serial();
  EXPECT_TRUE(serial.is_open_serial_);
}

TEST(SerialCommunicator2, ResetRepeatedlyDoesNotLeakDescriptors)
{
  PseudoTerminal pty;
  SerialCommunicator2 serial(pty.slave_path());
  ASSERT_TRUE(serial.is_open_serial_);

  // 次に開く fd の番号が増え続けないこと (開き直しのたびに閉じていること)
  int probe = dup(0);
  close(probe);
  for (int i = 0; i < 20; i++) {
    serial.reset_serial();
  }
  int probe_after = dup(0);
  close(probe_after);
  EXPECT_EQ(probe, probe_after);
}
