int MW_AHRS_Serial_Connect(char *portname_, uint32_t baudrate_);
bool AHRS_Read(unsigned char value[8]);
int MW_Serial_DisConnect();