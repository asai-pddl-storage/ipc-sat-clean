(define (problem strips-sat-x-1)
(:domain satellite)
(:objects
	satellite0 - satellite
	instrument0 - instrument
	instrument1 - instrument
	satellite1 - satellite
	instrument2 - instrument
	instrument3 - instrument
	satellite2 - satellite
	instrument4 - instrument
	instrument5 - instrument
	satellite3 - satellite
	instrument6 - instrument
	instrument7 - instrument
	satellite4 - satellite
	instrument8 - instrument
	satellite5 - satellite
	instrument9 - instrument
	satellite6 - satellite
	instrument10 - instrument
	instrument11 - instrument
	satellite7 - satellite
	instrument12 - instrument
	satellite8 - satellite
	instrument13 - instrument
	instrument14 - instrument
	satellite9 - satellite
	instrument15 - instrument
	satellite10 - satellite
	instrument16 - instrument
	instrument17 - instrument
	infrared14 - mode
	spectrograph10 - mode
	infrared13 - mode
	image11 - mode
	thermograph7 - mode
	spectrograph0 - mode
	image1 - mode
	image3 - mode
	image18 - mode
	spectrograph9 - mode
	thermograph15 - mode
	spectrograph16 - mode
	spectrograph17 - mode
	thermograph19 - mode
	spectrograph12 - mode
	thermograph2 - mode
	spectrograph8 - mode
	infrared6 - mode
	spectrograph4 - mode
	thermograph5 - mode
	Star1 - direction
	GroundStation0 - direction
	Planet2 - direction
	Planet3 - direction
	Star4 - direction
	Phenomenon5 - direction
	Phenomenon6 - direction
	Star7 - direction
	Planet8 - direction
	Phenomenon9 - direction
	Star10 - direction
	Phenomenon11 - direction
	Phenomenon12 - direction
	Planet13 - direction
	Star14 - direction
	Planet15 - direction
	Phenomenon16 - direction
	Star17 - direction
	Star18 - direction
	Star19 - direction
	Star20 - direction
	Phenomenon21 - direction
	Phenomenon22 - direction
	Planet23 - direction
	Phenomenon24 - direction
	Phenomenon25 - direction
	Star26 - direction
	Star27 - direction
	Star28 - direction
	Star29 - direction
	Star30 - direction
	Planet31 - direction
	Phenomenon32 - direction
	Phenomenon33 - direction
	Phenomenon34 - direction
	Planet35 - direction
	Star36 - direction
	Phenomenon37 - direction
	Planet38 - direction
	Star39 - direction
	Phenomenon40 - direction
	Star41 - direction
	Phenomenon42 - direction
	Star43 - direction
	Star44 - direction
	Phenomenon45 - direction
	Star46 - direction
	Star47 - direction
	Planet48 - direction
	Planet49 - direction
	Phenomenon50 - direction
	Planet51 - direction
	Planet52 - direction
	Planet53 - direction
	Star54 - direction
	Phenomenon55 - direction
	Planet56 - direction
	Phenomenon57 - direction
	Star58 - direction
	Planet59 - direction
	Phenomenon60 - direction
	Phenomenon61 - direction
	Star62 - direction
	Phenomenon63 - direction
	Star64 - direction
	Planet65 - direction
	Planet66 - direction
	Phenomenon67 - direction
	Phenomenon68 - direction
	Phenomenon69 - direction
	Planet70 - direction
	Star71 - direction
	Planet72 - direction
	Star73 - direction
	Planet74 - direction
	Star75 - direction
	Planet76 - direction
	Planet77 - direction
	Star78 - direction
	Star79 - direction
	Star80 - direction
	Star81 - direction
	Planet82 - direction
	Phenomenon83 - direction
	Planet84 - direction
	Planet85 - direction
	Planet86 - direction
	Phenomenon87 - direction
	Phenomenon88 - direction
	Planet89 - direction
	Star90 - direction
	Star91 - direction
	Phenomenon92 - direction
	Star93 - direction
	Star94 - direction
	Phenomenon95 - direction
	Star96 - direction
	Phenomenon97 - direction
	Planet98 - direction
	Planet99 - direction
	Planet100 - direction
	Phenomenon101 - direction
	Phenomenon102 - direction
	Phenomenon103 - direction
	Planet104 - direction
	Star105 - direction
	Star106 - direction
	Planet107 - direction
	Star108 - direction
	Planet109 - direction
	Phenomenon110 - direction
	Planet111 - direction
	Star112 - direction
	Phenomenon113 - direction
	Phenomenon114 - direction
	Phenomenon115 - direction
	Phenomenon116 - direction
	Star117 - direction
	Phenomenon118 - direction
	Planet119 - direction
	Phenomenon120 - direction
	Planet121 - direction
	Phenomenon122 - direction
	Star123 - direction
	Phenomenon124 - direction
	Planet125 - direction
	Star126 - direction
	Planet127 - direction
	Star128 - direction
	Phenomenon129 - direction
	Phenomenon130 - direction
	Star131 - direction
	Planet132 - direction
	Star133 - direction
	Phenomenon134 - direction
	Star135 - direction
	Planet136 - direction
	Phenomenon137 - direction
	Star138 - direction
	Planet139 - direction
	Star140 - direction
	Phenomenon141 - direction
	Planet142 - direction
	Star143 - direction
	Planet144 - direction
	Phenomenon145 - direction
	Phenomenon146 - direction
	Star147 - direction
	Star148 - direction
	Planet149 - direction
	Star150 - direction
	Phenomenon151 - direction
	Phenomenon152 - direction
	Planet153 - direction
	Phenomenon154 - direction
	Phenomenon155 - direction
	Phenomenon156 - direction
	Star157 - direction
	Star158 - direction
	Phenomenon159 - direction
	Phenomenon160 - direction
	Planet161 - direction
)
(:init
	(supports instrument0 spectrograph12)
	(supports instrument0 spectrograph17)
	(supports instrument0 spectrograph8)
	(calibration_target instrument0 Star1)
	(supports instrument1 infrared6)
	(calibration_target instrument1 Star1)
	(on_board instrument0 satellite0)
	(on_board instrument1 satellite0)
	(power_avail satellite0)
	(pointing satellite0 Phenomenon50)
	(supports instrument2 thermograph19)
	(calibration_target instrument2 GroundStation0)
	(supports instrument3 spectrograph4)
	(calibration_target instrument3 Star1)
	(on_board instrument2 satellite1)
	(on_board instrument3 satellite1)
	(power_avail satellite1)
	(pointing satellite1 Planet111)
	(supports instrument4 thermograph2)
	(calibration_target instrument4 GroundStation0)
	(supports instrument5 thermograph15)
	(supports instrument5 image3)
	(calibration_target instrument5 Star1)
	(on_board instrument4 satellite2)
	(on_board instrument5 satellite2)
	(power_avail satellite2)
	(pointing satellite2 Star10)
	(supports instrument6 thermograph15)
	(calibration_target instrument6 GroundStation0)
	(supports instrument7 thermograph15)
	(supports instrument7 image1)
	(supports instrument7 spectrograph0)
	(calibration_target instrument7 GroundStation0)
	(on_board instrument6 satellite3)
	(on_board instrument7 satellite3)
	(power_avail satellite3)
	(pointing satellite3 Phenomenon25)
	(supports instrument8 spectrograph16)
	(supports instrument8 spectrograph4)
	(supports instrument8 image3)
	(calibration_target instrument8 Star1)
	(on_board instrument8 satellite4)
	(power_avail satellite4)
	(pointing satellite4 Star29)
	(supports instrument9 spectrograph4)
	(supports instrument9 infrared6)
	(calibration_target instrument9 Star1)
	(on_board instrument9 satellite5)
	(power_avail satellite5)
	(pointing satellite5 Planet111)
	(supports instrument10 image18)
	(supports instrument10 spectrograph9)
	(calibration_target instrument10 Star1)
	(supports instrument11 spectrograph16)
	(supports instrument11 thermograph15)
	(supports instrument11 spectrograph9)
	(calibration_target instrument11 GroundStation0)
	(on_board instrument10 satellite6)
	(on_board instrument11 satellite6)
	(power_avail satellite6)
	(pointing satellite6 Planet74)
	(supports instrument12 thermograph19)
	(supports instrument12 spectrograph17)
	(calibration_target instrument12 Star1)
	(on_board instrument12 satellite7)
	(power_avail satellite7)
	(pointing satellite7 Star79)
	(supports instrument13 thermograph19)
	(calibration_target instrument13 Star1)
	(supports instrument14 thermograph2)
	(calibration_target instrument14 Star1)
	(on_board instrument13 satellite8)
	(on_board instrument14 satellite8)
	(power_avail satellite8)
	(pointing satellite8 Phenomenon24)
	(supports instrument15 thermograph5)
	(supports instrument15 thermograph2)
	(supports instrument15 spectrograph12)
	(calibration_target instrument15 GroundStation0)
	(on_board instrument15 satellite9)
	(power_avail satellite9)
	(pointing satellite9 Phenomenon156)
	(supports instrument16 infrared6)
	(supports instrument16 spectrograph8)
	(calibration_target instrument16 Star1)
	(supports instrument17 thermograph5)
	(supports instrument17 spectrograph4)
	(calibration_target instrument17 GroundStation0)
	(on_board instrument16 satellite10)
	(on_board instrument17 satellite10)
	(power_avail satellite10)
	(pointing satellite10 Phenomenon40)
)
(:goal (and
	(pointing satellite0 Phenomenon24)
	(pointing satellite1 Planet100)
	(pointing satellite4 Phenomenon159)
	(pointing satellite5 Planet49)
	(pointing satellite10 Star39)
	(have_image Planet2 spectrograph8)
	(have_image Planet2 spectrograph4)
	(have_image Planet2 spectrograph12)
	(have_image Planet2 spectrograph16)
	(have_image Planet2 thermograph19)
	(have_image Planet3 spectrograph17)
	(have_image Star4 thermograph2)
	(have_image Star4 image1)
	(have_image Star4 spectrograph0)
	(have_image Phenomenon5 spectrograph4)
	(have_image Phenomenon5 image1)
	(have_image Phenomenon5 image18)
	(have_image Phenomenon6 thermograph19)
	(have_image Phenomenon6 spectrograph12)
	(have_image Phenomenon6 thermograph2)
	(have_image Phenomenon6 spectrograph0)
	(have_image Phenomenon6 thermograph15)
	(have_image Star7 spectrograph16)
	(have_image Planet8 image1)
	(have_image Planet8 spectrograph0)
	(have_image Planet8 thermograph15)
	(have_image Phenomenon9 thermograph2)
	(have_image Phenomenon9 image3)
	(have_image Phenomenon9 thermograph15)
	(have_image Phenomenon9 thermograph19)
	(have_image Star10 thermograph5)
	(have_image Phenomenon11 thermograph5)
	(have_image Phenomenon11 thermograph19)
	(have_image Phenomenon11 infrared6)
	(have_image Phenomenon11 spectrograph16)
	(have_image Phenomenon11 spectrograph9)
	(have_image Phenomenon11 image18)
	(have_image Phenomenon12 spectrograph17)
	(have_image Phenomenon12 image3)
	(have_image Phenomenon12 spectrograph12)
	(have_image Phenomenon12 thermograph5)
	(have_image Phenomenon12 spectrograph9)
	(have_image Planet13 image18)
	(have_image Planet13 spectrograph9)
	(have_image Planet13 thermograph2)
	(have_image Star14 thermograph5)
	(have_image Star14 image18)
	(have_image Star14 spectrograph17)
	(have_image Star14 thermograph15)
	(have_image Planet15 thermograph5)
	(have_image Planet15 thermograph19)
	(have_image Planet15 thermograph2)
	(have_image Planet15 spectrograph12)
	(have_image Phenomenon16 spectrograph16)
	(have_image Phenomenon16 spectrograph8)
	(have_image Star17 infrared6)
	(have_image Star17 spectrograph12)
	(have_image Star17 spectrograph16)
	(have_image Star17 image18)
	(have_image Star18 spectrograph16)
	(have_image Star18 thermograph19)
	(have_image Star18 image18)
	(have_image Star20 infrared6)
	(have_image Star20 spectrograph9)
	(have_image Phenomenon21 spectrograph8)
	(have_image Phenomenon21 infrared6)
	(have_image Phenomenon21 spectrograph17)
	(have_image Phenomenon22 image3)
	(have_image Phenomenon22 thermograph19)
	(have_image Phenomenon24 image3)
	(have_image Phenomenon24 image18)
	(have_image Star26 spectrograph4)
	(have_image Star26 image3)
	(have_image Star27 thermograph2)
	(have_image Star27 image18)
	(have_image Star27 spectrograph8)
	(have_image Star27 image1)
	(have_image Star27 spectrograph17)
	(have_image Star28 thermograph15)
	(have_image Star29 image3)
	(have_image Planet31 thermograph19)
	(have_image Planet31 image3)
	(have_image Planet31 spectrograph12)
	(have_image Phenomenon32 spectrograph9)
	(have_image Phenomenon32 thermograph2)
	(have_image Phenomenon32 image1)
	(have_image Phenomenon32 image3)
	(have_image Phenomenon33 spectrograph17)
	(have_image Phenomenon33 spectrograph8)
	(have_image Phenomenon33 thermograph19)
	(have_image Phenomenon34 spectrograph17)
	(have_image Planet35 thermograph5)
	(have_image Planet35 image3)
	(have_image Planet35 spectrograph12)
	(have_image Planet35 spectrograph16)
	(have_image Star36 spectrograph16)
	(have_image Star36 spectrograph17)
	(have_image Star36 spectrograph8)
	(have_image Phenomenon37 spectrograph9)
	(have_image Phenomenon37 spectrograph0)
	(have_image Planet38 thermograph15)
	(have_image Planet38 infrared6)
	(have_image Star39 thermograph19)
	(have_image Star39 spectrograph12)
	(have_image Star41 spectrograph12)
	(have_image Star41 spectrograph8)
	(have_image Phenomenon42 infrared6)
	(have_image Phenomenon42 spectrograph12)
	(have_image Phenomenon42 image1)
	(have_image Phenomenon42 thermograph19)
	(have_image Star43 image3)
	(have_image Star43 spectrograph8)
	(have_image Star43 spectrograph0)
	(have_image Star43 thermograph15)
	(have_image Star46 image1)
	(have_image Star46 thermograph15)
	(have_image Star46 thermograph19)
	(have_image Star46 spectrograph8)
	(have_image Star46 thermograph5)
	(have_image Star47 infrared6)
	(have_image Planet48 thermograph5)
	(have_image Planet48 spectrograph17)
	(have_image Planet49 thermograph5)
	(have_image Planet49 spectrograph9)
	(have_image Planet49 thermograph15)
	(have_image Planet49 thermograph19)
	(have_image Phenomenon50 spectrograph17)
	(have_image Phenomenon50 spectrograph16)
	(have_image Phenomenon50 thermograph5)
	(have_image Planet51 image1)
	(have_image Planet51 spectrograph4)
	(have_image Planet51 infrared6)
	(have_image Planet51 image3)
	(have_image Planet52 image1)
	(have_image Planet53 spectrograph9)
	(have_image Planet53 spectrograph8)
	(have_image Star54 spectrograph9)
	(have_image Phenomenon55 thermograph15)
	(have_image Phenomenon55 thermograph19)
	(have_image Planet56 thermograph2)
	(have_image Phenomenon57 spectrograph4)
	(have_image Phenomenon57 spectrograph12)
	(have_image Phenomenon57 spectrograph16)
	(have_image Phenomenon57 thermograph15)
	(have_image Phenomenon57 spectrograph17)
	(have_image Star58 image3)
	(have_image Star58 thermograph15)
	(have_image Star58 spectrograph16)
	(have_image Star58 image1)
	(have_image Star58 infrared6)
	(have_image Planet59 spectrograph17)
	(have_image Planet59 spectrograph12)
	(have_image Planet59 thermograph5)
	(have_image Phenomenon60 image18)
	(have_image Phenomenon60 spectrograph4)
	(have_image Phenomenon60 thermograph2)
	(have_image Phenomenon60 spectrograph16)
	(have_image Phenomenon61 spectrograph12)
	(have_image Phenomenon61 spectrograph16)
	(have_image Phenomenon61 infrared6)
	(have_image Phenomenon61 spectrograph0)
	(have_image Star62 spectrograph16)
	(have_image Star62 thermograph5)
	(have_image Star62 image18)
	(have_image Phenomenon63 spectrograph12)
	(have_image Phenomenon63 spectrograph17)
	(have_image Phenomenon63 image1)
	(have_image Phenomenon63 thermograph15)
	(have_image Star64 spectrograph16)
	(have_image Star64 thermograph19)
	(have_image Star64 infrared6)
	(have_image Star64 thermograph15)
	(have_image Star64 image1)
	(have_image Planet65 spectrograph17)
	(have_image Planet66 image3)
	(have_image Planet66 spectrograph16)
	(have_image Phenomenon67 thermograph5)
	(have_image Phenomenon67 thermograph19)
	(have_image Phenomenon67 spectrograph16)
	(have_image Phenomenon67 thermograph15)
	(have_image Phenomenon67 image3)
	(have_image Phenomenon68 spectrograph8)
	(have_image Phenomenon68 spectrograph17)
	(have_image Phenomenon68 image3)
	(have_image Planet70 thermograph15)
	(have_image Planet70 spectrograph9)
	(have_image Planet70 thermograph2)
	(have_image Planet72 thermograph2)
	(have_image Star73 spectrograph17)
	(have_image Star73 spectrograph4)
	(have_image Star73 infrared6)
	(have_image Planet74 infrared6)
	(have_image Planet74 spectrograph0)
	(have_image Star75 spectrograph9)
	(have_image Star75 infrared6)
	(have_image Star75 spectrograph17)
	(have_image Planet77 infrared6)
	(have_image Star78 spectrograph16)
	(have_image Star79 spectrograph12)
	(have_image Star80 thermograph2)
	(have_image Star80 thermograph15)
	(have_image Star80 image1)
	(have_image Star81 image3)
	(have_image Star81 thermograph15)
	(have_image Star81 thermograph19)
	(have_image Star81 spectrograph0)
	(have_image Star81 spectrograph16)
	(have_image Star81 infrared6)
	(have_image Planet82 spectrograph17)
	(have_image Planet82 thermograph15)
	(have_image Planet82 spectrograph4)
	(have_image Planet82 image1)
	(have_image Phenomenon83 spectrograph17)
	(have_image Planet84 thermograph15)
	(have_image Planet84 spectrograph17)
	(have_image Planet84 spectrograph12)
	(have_image Planet84 spectrograph9)
	(have_image Planet85 image1)
	(have_image Phenomenon87 thermograph2)
	(have_image Phenomenon87 thermograph15)
	(have_image Phenomenon87 thermograph19)
	(have_image Phenomenon88 spectrograph8)
	(have_image Phenomenon88 spectrograph12)
	(have_image Phenomenon88 thermograph19)
	(have_image Planet89 thermograph19)
	(have_image Star90 spectrograph4)
	(have_image Star90 thermograph15)
	(have_image Star90 spectrograph17)
	(have_image Star91 spectrograph9)
	(have_image Phenomenon92 thermograph5)
	(have_image Phenomenon92 image3)
	(have_image Phenomenon92 thermograph15)
	(have_image Phenomenon92 thermograph2)
	(have_image Star94 spectrograph17)
	(have_image Star94 spectrograph8)
	(have_image Star94 thermograph19)
	(have_image Star94 thermograph2)
	(have_image Phenomenon95 spectrograph8)
	(have_image Phenomenon95 image3)
	(have_image Phenomenon95 spectrograph16)
	(have_image Phenomenon95 spectrograph4)
	(have_image Star96 image3)
	(have_image Star96 thermograph2)
	(have_image Star96 image18)
	(have_image Phenomenon97 spectrograph0)
	(have_image Phenomenon97 spectrograph12)
	(have_image Phenomenon97 thermograph19)
	(have_image Planet98 thermograph2)
	(have_image Planet99 thermograph19)
	(have_image Planet99 thermograph15)
	(have_image Planet99 spectrograph17)
	(have_image Planet99 infrared6)
	(have_image Phenomenon101 thermograph5)
	(have_image Phenomenon101 spectrograph12)
	(have_image Phenomenon102 image1)
	(have_image Phenomenon103 spectrograph17)
	(have_image Phenomenon103 thermograph5)
	(have_image Planet104 spectrograph16)
	(have_image Planet104 thermograph19)
	(have_image Star105 thermograph5)
	(have_image Star105 spectrograph9)
	(have_image Star105 image3)
	(have_image Star105 spectrograph4)
	(have_image Star108 thermograph5)
	(have_image Star108 thermograph2)
	(have_image Star108 image1)
	(have_image Planet109 infrared6)
	(have_image Planet109 thermograph2)
	(have_image Planet109 image1)
	(have_image Phenomenon110 spectrograph0)
	(have_image Phenomenon110 thermograph2)
	(have_image Phenomenon110 spectrograph16)
	(have_image Phenomenon110 image18)
	(have_image Planet111 thermograph19)
	(have_image Planet111 spectrograph4)
	(have_image Phenomenon113 spectrograph9)
	(have_image Phenomenon113 image3)
	(have_image Phenomenon114 thermograph15)
	(have_image Phenomenon114 image1)
	(have_image Phenomenon115 spectrograph9)
	(have_image Planet119 spectrograph17)
	(have_image Planet119 image1)
	(have_image Planet119 spectrograph9)
	(have_image Phenomenon120 spectrograph16)
	(have_image Phenomenon120 thermograph19)
	(have_image Phenomenon120 infrared6)
	(have_image Planet121 spectrograph17)
	(have_image Phenomenon122 spectrograph9)
	(have_image Phenomenon122 thermograph15)
	(have_image Star123 spectrograph0)
	(have_image Star123 spectrograph12)
	(have_image Star123 thermograph2)
	(have_image Star123 thermograph19)
	(have_image Phenomenon124 spectrograph17)
	(have_image Phenomenon124 thermograph15)
	(have_image Phenomenon124 spectrograph16)
	(have_image Planet125 spectrograph8)
	(have_image Planet125 spectrograph16)
	(have_image Planet125 thermograph19)
	(have_image Planet125 image1)
	(have_image Star126 spectrograph12)
	(have_image Star126 thermograph2)
	(have_image Star126 thermograph15)
	(have_image Planet127 spectrograph0)
	(have_image Planet127 thermograph5)
	(have_image Planet127 spectrograph9)
	(have_image Planet127 image18)
	(have_image Planet127 thermograph19)
	(have_image Planet127 spectrograph4)
	(have_image Star128 image18)
	(have_image Star128 spectrograph4)
	(have_image Star128 spectrograph16)
	(have_image Star128 spectrograph8)
	(have_image Star128 spectrograph12)
	(have_image Phenomenon130 spectrograph17)
	(have_image Phenomenon130 spectrograph16)
	(have_image Phenomenon130 image18)
	(have_image Phenomenon130 thermograph5)
	(have_image Star133 spectrograph4)
	(have_image Star133 spectrograph9)
	(have_image Star133 spectrograph8)
	(have_image Phenomenon134 infrared6)
	(have_image Phenomenon134 thermograph19)
	(have_image Star135 infrared6)
	(have_image Star135 thermograph19)
	(have_image Star135 spectrograph0)
	(have_image Star135 spectrograph8)
	(have_image Planet136 spectrograph12)
	(have_image Planet136 thermograph2)
	(have_image Planet136 spectrograph16)
	(have_image Phenomenon137 thermograph5)
	(have_image Star138 infrared6)
	(have_image Star138 thermograph2)
	(have_image Star138 spectrograph4)
	(have_image Star138 thermograph19)
	(have_image Planet139 thermograph15)
	(have_image Star140 infrared6)
	(have_image Star140 image3)
	(have_image Star140 spectrograph12)
	(have_image Star140 spectrograph0)
	(have_image Planet142 image3)
	(have_image Star143 spectrograph4)
	(have_image Star143 infrared6)
	(have_image Star143 thermograph19)
	(have_image Star143 image1)
	(have_image Star143 thermograph15)
	(have_image Phenomenon146 spectrograph0)
	(have_image Phenomenon146 thermograph5)
	(have_image Phenomenon146 thermograph15)
	(have_image Phenomenon146 spectrograph16)
	(have_image Star147 thermograph15)
	(have_image Planet149 spectrograph16)
	(have_image Phenomenon151 image18)
	(have_image Phenomenon152 thermograph15)
	(have_image Phenomenon152 spectrograph12)
	(have_image Phenomenon152 image18)
	(have_image Planet153 thermograph19)
	(have_image Phenomenon154 thermograph15)
	(have_image Phenomenon154 spectrograph12)
	(have_image Phenomenon154 spectrograph8)
	(have_image Phenomenon154 infrared6)
	(have_image Phenomenon154 spectrograph4)
	(have_image Phenomenon155 spectrograph17)
	(have_image Phenomenon155 spectrograph8)
	(have_image Phenomenon155 image3)
	(have_image Star157 infrared6)
	(have_image Star157 image18)
	(have_image Star157 thermograph2)
	(have_image Star157 thermograph19)
	(have_image Star157 spectrograph4)
	(have_image Star158 infrared6)
	(have_image Star158 spectrograph17)
	(have_image Phenomenon159 image1)
	(have_image Phenomenon159 spectrograph4)
	(have_image Phenomenon159 spectrograph16)
	(have_image Phenomenon159 thermograph2)
	(have_image Phenomenon160 thermograph2)
	(have_image Phenomenon160 spectrograph0)
	(have_image Phenomenon160 thermograph5)
	(have_image Planet161 spectrograph16)
	(have_image Planet161 spectrograph0)
	(have_image Planet161 infrared6)
	(have_image Planet161 thermograph15)
))

)