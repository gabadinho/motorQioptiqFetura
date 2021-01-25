s/^SUPPORT=/SUPPORT=\/usr\/local\/epics\/modules\/synApps\/support/
s/^EPICS_BASE=/EPICS_BASE=\/usr\/local\/epics\/base/
/^-include\|^SUPPORT\|^EPICS_BASE\|^ASYN\|^AUTOSAVE\|^BUSY\|^CALC\|^DEVIOCSTATS\|^MOTOR\|^SSCAN\|^STREAM\|^SNCSEQ/! s/^/#/

