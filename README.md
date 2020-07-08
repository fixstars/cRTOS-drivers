# Supporting Linux drivers for cRTOS

This contains modified Jailhouse ivshmem v2 Linux drivers.
The following have been done:
 * Decouple the dependencies on Jailhouse Linux kernel source

This are created for Linux v5.4, but with subtle changes, one might port to any other versions.
Reference the Jailhouse upstream drivers for the required changes.

It contains 3 different modules
 * uio_ivshmem
 * ivshmem_net
 * shadow

The first 2 are identical to Jailhouse's upstream driver, but can be compiled for vanilla Linux.
The `uio_ivshmem` driver maps a ivshmem v2 devices as a uio device.
The `ivshmem_net` is for inter-cell Ethernet.

The `shadow` is a custom rolled device driver for remoce system call of cRTOS.
It is based on the `ivshmem_net` driver, utilizing 2 vrings to send messages from/to the RTOS.
This interface is exposed as a character device.
In addition, it provides interface to map the ivshmem region on page basis (the `uio_ivshmem` maps the whole region at once).
This is used to provide an identical memory space of the remote real-time process for the Proxy process in Linux.
This is exposed on the mmap interface of the character device.

## Requirements

 * gcc
 * Linux kernel build environment for kernel modules

## To Build

Execute the following commands in the root directory.

```sh
make
```

## To Use

`uio_ivshmem` and `ivshmem_net` can simply be loaded with `insmod`

For example:
```sh
insmod ivshmem_net
```

On the other hand, the shadow device takes a argument pointing the physical starting address of ivshmem region.
This is done to allow the shared memory region to exist not in between the state table and I/O regions.
For more information, reference the Jailhuose ivshmem v2 specification.

For example:
```
# The R/W shared memory region atarts at physical address 0x108000000
insmod shadow.ko shmaddr=0x108000000
```

## LICENSE

This program is licensed under GPL version 2.

You can find a copy of the license in the `LICENSE` file.
