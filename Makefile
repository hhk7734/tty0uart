rwildcard=$(wildcard $1$2) $(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2))

.PHONY: all
all:
	make -C module

.PHONY: install
install:
	make -C module modules_install
	sed -e ':a' -e 'N' -e '$$!ba' -e 's/\ntty0uart//g' -i /etc/modules
	echo tty0uart | tee -a /etc/modules
	depmod
	install -m 0644 51-tty0uart.rules /etc/udev/rules.d
	modprobe tty0uart ||\
		( sed -e ':a' -e 'N' -e '$$!ba' -e 's/\ntty0uart//g' -i /etc/modules &&\
		rm -f /etc/udev/rules.d/51-tty0uart.rules )

.PHONY: clean
clean:
	make -C module clean

.PHONY: distclean
distclean: clean

.PHONY: uninstall
uninstall:
	modprobe -r tty0uart || true
	depmod
	sed -e ':a' -e 'N' -e '$$!ba' -e 's/\ntty0uart//g' -i /etc/modules
	rm -f /lib/modules/$(shell uname -r)/*/tty0uart.ko
	rm -f /etc/udev/rules.d/51-tty0uart.rules

.PHONY: clang
clang: $(call rwildcard,,*.c) $(call rwildcard,,*.cpp) $(call rwildcard,,*.h) $(call rwildcard,,*.hpp)
	clang-format -style=file -i -verbose $^