obj-m := kedei62.o


all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	rm -f kedei62.dtbo

dtbo: kedei62.dtbo

kedei62.dtbo: kedei62.dts
	dtc -@ -I dts -O dtb -o kedei62.dtbo kedei62.dts
