//This script is used to initialize gmi script links
//and set up gmi dll calls
//
//Now auto-called from Gmn_Init();
var dllpath;
dllpath="GMNewton.dll";
global.__DLLInit__ = external_define(dllpath,"DLLInit",dll_cdecl,ty_real,0);
external_call(global.__DLLInit__);
global.__set_script_transform_body__ = external_define(dllpath,"set_script_transform_body",dll_cdecl,ty_real,1,ty_real);
gmi_set_script_transform_body(gmi_update_body);
