

1���ļ���RFID������Դ�룬�����ַ�����ģ��
2�����ļ����뵽�ں�·�� drivers/misc/ , ���޸Ķ�Ӧ��Makefile Kconfig�ļ���������ģ�鼴�ɡ�
3��ʹ�ã�
	fd = open("/dev/io", O_RDWR);
	ret = ioctl(fd, IO_IDCARD_IN, &card_id);