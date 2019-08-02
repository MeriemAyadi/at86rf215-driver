#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
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
__used
__attribute__((section("__versions"))) = {
	{ 0x367398b6, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0x62b59d7e, __VMLINUX_SYMBOL_STR(driver_unregister) },
	{ 0xca007fb9, __VMLINUX_SYMBOL_STR(__spi_register_driver) },
	{ 0xc84dc054, __VMLINUX_SYMBOL_STR(ieee802154_free_hw) },
	{ 0xf90d918d, __VMLINUX_SYMBOL_STR(of_property_read_variable_u8_array) },
	{ 0x2ccf0318, __VMLINUX_SYMBOL_STR(of_get_named_gpio_flags) },
	{ 0xe49a8c4e, __VMLINUX_SYMBOL_STR(dev_err) },
	{ 0x2e5810c6, __VMLINUX_SYMBOL_STR(__aeabi_unwind_cpp_pr1) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0xb1ad28e0, __VMLINUX_SYMBOL_STR(__gnu_mcount_nc) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=mac802154";

MODULE_ALIAS("spi:at86rf215");
MODULE_ALIAS("of:N*T*Catmel,at86rf215");
MODULE_ALIAS("of:N*T*Catmel,at86rf215C*");

MODULE_INFO(srcversion, "651D4AEB16F852EE1E08569");
