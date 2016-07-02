#3DS_IP := 10.1.1.28
3DS_IP := 192.168.6.215

all: upload

clean:
	$(MAKE) -C build clean
	
binary:
	$(MAKE) -C build all
	cp build/*.3dsx .
	
upload: binary
	3dslink -a $(3DS_IP) *.3dsx
	
	
