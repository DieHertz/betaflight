ifeq ($(TARGET), SPEDIXF4)
F405_TARGETS   += $(TARGET)
else
ifeq ($(TARGET), SPEDIXF7)
F7X2RE_TARGETS += $(TARGET)
endif
endif
FEATURES       = VCP ONBOARDFLASH
TARGET_SRC = \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/compass/compass_qmc5883l.c \
            drivers/max7456.c
