
// AUTO GENERATED by vnxcppcodegen

#include <neo_relayboard_v3/RelayBoardV3Base.hxx>

#include <neo_relayboard_v3/package.hxx>
#include <vnx/vnx.h>



namespace vnx {


} // namespace vnx


namespace neo_relayboard_v3 {


static void register_all_types() {
	vnx::register_type_code(::neo_relayboard_v3::RelayBoardV3Base::static_create_type_code());
}

static struct vnx_static_init {
	vnx_static_init() {
		register_all_types();
	}
} vnx_static_init_;

const vnx::TypeCode* const vnx_native_type_code_RelayBoardV3Base = vnx::get_type_code(vnx::Hash64(0x59c86a8cb3f030daull));

} // namespace neo_relayboard_v3
