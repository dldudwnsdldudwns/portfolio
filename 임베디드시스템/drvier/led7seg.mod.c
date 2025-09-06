#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/export-internal.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

#ifdef CONFIG_UNWINDER_ORC
#include <asm/orc_header.h>
ORC_HEADER;
#endif

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif



static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0xae7b72ae, "module_put" },
	{ 0x69a20af4, "device_destroy" },
	{ 0x15b3ce7d, "class_destroy" },
	{ 0x6bc3fbc0, "__unregister_chrdev" },
	{ 0xedc03953, "iounmap" },
	{ 0x122c3a7e, "_printk" },
	{ 0xeae3dfd6, "__const_udelay" },
	{ 0xa65c6def, "alt_cb_patch_nops" },
	{ 0x7ef09b2d, "try_module_get" },
	{ 0x5fd63079, "__register_chrdev" },
	{ 0xb0c6c225, "class_create" },
	{ 0xfeae87a5, "device_create" },
	{ 0xaf56600a, "arm64_use_ng_mappings" },
	{ 0x40863ba1, "ioremap_prot" },
	{ 0x6f6ab014, "module_layout" },
};

MODULE_INFO(depends, "");


MODULE_INFO(srcversion, "A56899E65BA3896553C8BE9");
