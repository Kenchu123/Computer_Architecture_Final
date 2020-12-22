if ( $?prompt ) then

###################################################################
# Cadence                                                         #
###################################################################
	source /usr/cad/cadence/cshrc				#For Verdi ncverilog 
###################################################################
# Calibre                                                         #
###################################################################
	source /usr/mentor/CIC/calibre.cshrc
###################################################################
# Debussy $ Verdi                                                 #
###################################################################
#	source /usr/spring_soft/CIC/debussy.cshrc 
	source /usr/spring_soft/CIC/verdi.cshrc 	#For nLint

	source /usr/cad/synopsys/CIC/primetime.cshrc
	source /usr/cadence/CIC/confrml.cshrc
#	source /usr/cad/cadence/CIC/edi.cshrc
###################################################################
# Synopsys                                                        #
###################################################################
	source /usr/cad/synopsys/CIC/synthesis.cshrc
#	source /usr/cad/synopsys/CIC/hspice.cshrc
#	source /usr/cad/synopsys/CIC/cosmos_scope.cshrc
###################################################################
# Cadence Formal                                                  #
###################################################################
#	source /usr/cad/cadence/jasper/cur/cshrc
###################################################################
# Innovus                                                         #
###################################################################
	source /usr/cad/innovus/CIC/innovus.cshrc #For innovus
	source /usr/cad/innovus/CIC/license.cshrc

set path=(/usr/cad/synopsys/customexplorer/cur/bin $path)
set path=(/usr/cad/spring_soft/laker/cur/bin $path)

endif
