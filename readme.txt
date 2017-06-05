

1、文件是RFID驱动的源码，采用字符驱动模型
2、将文件放入到内核路径 drivers/misc/ , 并修改对应得Makefile Kconfig文件将其编译成模块即可。
3、使用：
	fd = open("/dev/io", O_RDWR);
	ret = ioctl(fd, IO_IDCARD_IN, &card_id);