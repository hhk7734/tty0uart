rwildcard=$(wildcard $1$2) $(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2))

.PHONY: all
all:
	make -C module

.PHONY: install
install:

.PHONY: clean
clean:
	make -C module clean

.PHONY: distclean
distclean: clean

.PHONY: uninstall
uninstall:

.PHONY: clang
clang: $(call rwildcard,,*.c) $(call rwildcard,,*.cpp) $(call rwildcard,,*.h) $(call rwildcard,,*.hpp)
	clang-format -style=file -i -verbose $^