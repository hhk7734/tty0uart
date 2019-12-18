# tty0uart

**tty0uart**: Null-modem emulator connecting virtual tty to virtual UART

/dev/ttyvs0 \<--\> /dev/ttyVS0

Various programs can detect /dev/ttyVS0 but usually not /dev/ttyvs0.

## Installation

```bash
git clone https://github.com/hhk7734/tty0uart.git
```

```bash
cd tty0uart
```

```bash
make &&
sudo make install
```

## Removal

```bash
cd tty0uart
```

```bash
sudo make uninstall
```
