#include <tcl.h>
#include <tclreadline.h>
#include <tclExtend.h>
#include <iostream>

// Minimal example of Tcl shell
// Note that man is enabled only on Tclx. So we need to enable it
int tclAppInit(Tcl_Interp* interp)
{
  return 0;
}

int main(int argc, char* argv[]){
    // Setup the app with tcl
    auto* interp = Tcl_CreateInterp();
    Tcl_Init(interp);
    Tcl_Main(1, argv, tclAppInit);
}
