INCLUDES = -I. -I/usr/X11R6/include/ -std=c++0x

#SUBDIRS = smovableobjects fitnessfunctions sensors renders     \
#	  controllers actuators arenas populations experiments \
#	  collisions commoninterface emitters

bin_PROGRAMS  = bioinstsim2
bioinstsim2_SOURCES = main.cpp simobject.cpp agent.cpp simulator.cpp arena.cpp bioinstsim.cpp random.cpp circulararena.cpp rectangulararena.cpp boundlessarena.cpp common.cpp openglrender.cpp render.cpp arguments.cpp experiment.cpp populationanalyzer.cpp crminrobotagent.cpp celldatacontainers.cpp crminrobotagent_optimised.cpp thresholdinrobotagent_optimised.cpp ctrnninrobotagent.cpp lineqinrobotagent.cpp robotagent.cpp robotagent_optimised.cpp behavior.cpp aggregatebehavior.cpp testexperiment.cpp dispersebehavior.cpp flockbehavior.cpp homingbehavior.cpp randomwalkbehavior.cpp circlebehavior.cpp stopbehavior.cpp featurevector.cpp sbottrace.cpp stopwhenclosetootheragentbehavior.cpp

bioinstsim2_LDADD   = -lGL -lGLU -lX11 -L/usr/X11R6/lib -L/usr/lib  
#bioinstsim2_LDFLAGS  = -pthread

EXTRA_DIST = main.h simobject.h agent.h simulator.h arena.h bioinstsim.h random.h circulararena.h rectangulararena.h boundlessarena.h common.h openglrender.h render.h arguments.h experiment.h populationanalyzer.h crminrobotagent.h celldatacontainers.h crminrobotagent_optimised.h thresholdinrobotagent_optimised.h ctrnninrobotagent.h lineqinrobotagent.h robotagent.h robotagent_optimised.h behavior.h aggregatebehavior.h testexperiment.h dispersebehavior.h flockbehavior.h homingbehavior.h randomwalkbehavior.h circlebehavior.h stopbehavior.h featurevector.h sbottrace.h stopwhenclosetootheragentbehavior.h

