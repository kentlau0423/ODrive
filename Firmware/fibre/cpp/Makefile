
# pkgconf --cflags libusb
# pkgconf --libs libusb

linux:
	g++ -shared -o libfibre.so -fPIC -std=c++17 -I/usr/include/libusb-1.0 -I./include -DFIBRE_COMPILE -DFIBRE_ENABLE_CLIENT \
		libfibre.cpp \
		platform_support/libusb_transport.cpp \
		legacy_protocol.cpp \
		legacy_object_client.cpp \
		logging.cpp \
		-lusb

windows:
	g++ -shared -o libfibre.dll -fPIC -std=c++17 -I./libusb-1.0.23/include/libusb-1.0 -I./include -DFIBRE_COMPILE -DFIBRE_ENABLE_CLIENT \
		libfibre.cpp \
		platform_support/libusb_transport.cpp \
		legacy_protocol.cpp \
		legacy_object_client.cpp \
		logging.cpp \
		-static-libgcc -Wl,-Bstatic -lstdc++ -lpthread libusb-1.0.23/MinGW64/static/libusb-1.0.a -Wl,-Bdynamic

# nm -D libfibre.so | grep T
