#include "common.h"

#include "bioinstsim.h"
#include <ctime>
#include <time.h> 



/******************************************************************************/
/******************************************************************************/

CSimulator* GetSimulator();

/******************************************************************************/
/******************************************************************************/

int main(int argc, char** argv)
{
    DEBUGOUT("BioInstSim is starting...\n");

    clock_t t1=clock();
  
    CBioInstSim cBioInstSim(argc, argv);
    cBioInstSim.Run();

    clock_t t2=clock();

    fflush(stdout);
    fprintf(stderr, "\nSimulation time: %5.2f seconds\n", double(t2-t1) / CLOCKS_PER_SEC);
    
    DEBUGOUT("The BioInstSim stopping...\n");

    return 0;
}

/******************************************************************************/
/******************************************************************************/
