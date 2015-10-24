(define (problem os-sequencedstrips-p170_3)
(:domain openstacks-sequencedstrips-nonADL-nonNegated)
(:objects 
n0 n1 n2 n3 n4 n5 n6 n7 n8 n9 n10 n11 n12 n13 n14 n15 n16 n17 n18 n19 n20 n21 n22 n23 n24 n25 n26 n27 n28 n29 n30 n31 n32 n33 n34 n35 n36 n37 n38 n39 n40 n41 n42 n43 n44 n45 n46 n47 n48 n49 n50 n51 n52 n53 n54 n55 n56 n57 n58 n59 n60 n61 n62 n63 n64 n65 n66 n67 n68 n69 n70 n71 n72 n73 n74 n75 n76 n77 n78 n79 n80 n81 n82 n83 n84 n85 n86 n87 n88 n89 n90 n91 n92 n93 n94 n95 n96 n97 n98 n99 n100 n101 n102 n103 n104 n105 n106 n107 n108 n109 n110 n111 n112 n113 n114 n115 n116 n117 n118 n119 n120 n121 n122 n123 n124 n125 n126 n127 n128 n129 n130 n131 n132 n133 n134 n135 n136 n137 n138 n139 n140 n141 n142 n143 n144 n145 n146 n147 n148 n149 n150 n151 n152 n153 n154 n155 n156 n157 n158 n159 n160 n161 n162 n163 n164 n165 n166 n167 n168 n169 n170  - count
)

(:init
(next-count n0 n1) (next-count n1 n2) (next-count n2 n3) (next-count n3 n4) (next-count n4 n5) (next-count n5 n6) (next-count n6 n7) (next-count n7 n8) (next-count n8 n9) (next-count n9 n10) (next-count n10 n11) (next-count n11 n12) (next-count n12 n13) (next-count n13 n14) (next-count n14 n15) (next-count n15 n16) (next-count n16 n17) (next-count n17 n18) (next-count n18 n19) (next-count n19 n20) (next-count n20 n21) (next-count n21 n22) (next-count n22 n23) (next-count n23 n24) (next-count n24 n25) (next-count n25 n26) (next-count n26 n27) (next-count n27 n28) (next-count n28 n29) (next-count n29 n30) (next-count n30 n31) (next-count n31 n32) (next-count n32 n33) (next-count n33 n34) (next-count n34 n35) (next-count n35 n36) (next-count n36 n37) (next-count n37 n38) (next-count n38 n39) (next-count n39 n40) (next-count n40 n41) (next-count n41 n42) (next-count n42 n43) (next-count n43 n44) (next-count n44 n45) (next-count n45 n46) (next-count n46 n47) (next-count n47 n48) (next-count n48 n49) (next-count n49 n50) (next-count n50 n51) (next-count n51 n52) (next-count n52 n53) (next-count n53 n54) (next-count n54 n55) (next-count n55 n56) (next-count n56 n57) (next-count n57 n58) (next-count n58 n59) (next-count n59 n60) (next-count n60 n61) (next-count n61 n62) (next-count n62 n63) (next-count n63 n64) (next-count n64 n65) (next-count n65 n66) (next-count n66 n67) (next-count n67 n68) (next-count n68 n69) (next-count n69 n70) (next-count n70 n71) (next-count n71 n72) (next-count n72 n73) (next-count n73 n74) (next-count n74 n75) (next-count n75 n76) (next-count n76 n77) (next-count n77 n78) (next-count n78 n79) (next-count n79 n80) (next-count n80 n81) (next-count n81 n82) (next-count n82 n83) (next-count n83 n84) (next-count n84 n85) (next-count n85 n86) (next-count n86 n87) (next-count n87 n88) (next-count n88 n89) (next-count n89 n90) (next-count n90 n91) (next-count n91 n92) (next-count n92 n93) (next-count n93 n94) (next-count n94 n95) (next-count n95 n96) (next-count n96 n97) (next-count n97 n98) (next-count n98 n99) (next-count n99 n100) (next-count n100 n101) (next-count n101 n102) (next-count n102 n103) (next-count n103 n104) (next-count n104 n105) (next-count n105 n106) (next-count n106 n107) (next-count n107 n108) (next-count n108 n109) (next-count n109 n110) (next-count n110 n111) (next-count n111 n112) (next-count n112 n113) (next-count n113 n114) (next-count n114 n115) (next-count n115 n116) (next-count n116 n117) (next-count n117 n118) (next-count n118 n119) (next-count n119 n120) (next-count n120 n121) (next-count n121 n122) (next-count n122 n123) (next-count n123 n124) (next-count n124 n125) (next-count n125 n126) (next-count n126 n127) (next-count n127 n128) (next-count n128 n129) (next-count n129 n130) (next-count n130 n131) (next-count n131 n132) (next-count n132 n133) (next-count n133 n134) (next-count n134 n135) (next-count n135 n136) (next-count n136 n137) (next-count n137 n138) (next-count n138 n139) (next-count n139 n140) (next-count n140 n141) (next-count n141 n142) (next-count n142 n143) (next-count n143 n144) (next-count n144 n145) (next-count n145 n146) (next-count n146 n147) (next-count n147 n148) (next-count n148 n149) (next-count n149 n150) (next-count n150 n151) (next-count n151 n152) (next-count n152 n153) (next-count n153 n154) (next-count n154 n155) (next-count n155 n156) (next-count n156 n157) (next-count n157 n158) (next-count n158 n159) (next-count n159 n160) (next-count n160 n161) (next-count n161 n162) (next-count n162 n163) (next-count n163 n164) (next-count n164 n165) (next-count n165 n166) (next-count n166 n167) (next-count n167 n168) (next-count n168 n169) (next-count n169 n170) 
(stacks-avail n0)

(waiting o1)
(includes o1 p14)(includes o1 p113)(includes o1 p117)

(waiting o2)
(includes o2 p23)(includes o2 p36)(includes o2 p55)(includes o2 p75)(includes o2 p87)(includes o2 p112)(includes o2 p120)

(waiting o3)
(includes o3 p19)(includes o3 p34)(includes o3 p47)(includes o3 p127)(includes o3 p138)

(waiting o4)
(includes o4 p25)(includes o4 p72)(includes o4 p93)(includes o4 p132)

(waiting o5)
(includes o5 p18)(includes o5 p27)(includes o5 p61)(includes o5 p63)(includes o5 p70)(includes o5 p114)

(waiting o6)
(includes o6 p1)(includes o6 p15)(includes o6 p20)(includes o6 p98)(includes o6 p144)(includes o6 p153)

(waiting o7)
(includes o7 p50)(includes o7 p80)(includes o7 p102)(includes o7 p163)

(waiting o8)
(includes o8 p19)(includes o8 p40)(includes o8 p51)(includes o8 p111)(includes o8 p148)(includes o8 p169)

(waiting o9)
(includes o9 p3)(includes o9 p27)(includes o9 p120)(includes o9 p132)

(waiting o10)
(includes o10 p24)(includes o10 p114)(includes o10 p144)

(waiting o11)
(includes o11 p1)(includes o11 p3)(includes o11 p10)(includes o11 p106)(includes o11 p119)

(waiting o12)
(includes o12 p28)(includes o12 p84)(includes o12 p103)(includes o12 p130)(includes o12 p151)(includes o12 p155)(includes o12 p160)

(waiting o13)
(includes o13 p12)(includes o13 p24)(includes o13 p103)(includes o13 p121)(includes o13 p123)(includes o13 p127)(includes o13 p151)

(waiting o14)
(includes o14 p135)(includes o14 p142)(includes o14 p157)

(waiting o15)
(includes o15 p17)(includes o15 p49)(includes o15 p109)(includes o15 p143)

(waiting o16)
(includes o16 p4)(includes o16 p15)(includes o16 p17)(includes o16 p19)(includes o16 p21)(includes o16 p72)(includes o16 p142)(includes o16 p146)

(waiting o17)
(includes o17 p9)(includes o17 p46)(includes o17 p68)(includes o17 p128)(includes o17 p144)(includes o17 p148)(includes o17 p168)

(waiting o18)
(includes o18 p88)(includes o18 p95)(includes o18 p119)(includes o18 p136)(includes o18 p138)(includes o18 p139)(includes o18 p153)(includes o18 p161)(includes o18 p165)

(waiting o19)
(includes o19 p42)(includes o19 p57)(includes o19 p75)(includes o19 p79)(includes o19 p82)(includes o19 p156)

(waiting o20)
(includes o20 p70)(includes o20 p100)(includes o20 p104)(includes o20 p158)(includes o20 p159)(includes o20 p160)(includes o20 p161)

(waiting o21)
(includes o21 p8)(includes o21 p25)(includes o21 p33)(includes o21 p71)(includes o21 p145)(includes o21 p155)

(waiting o22)
(includes o22 p98)(includes o22 p167)

(waiting o23)
(includes o23 p37)(includes o23 p48)(includes o23 p87)(includes o23 p91)(includes o23 p116)(includes o23 p142)(includes o23 p158)

(waiting o24)
(includes o24 p76)(includes o24 p83)(includes o24 p112)(includes o24 p153)

(waiting o25)
(includes o25 p22)(includes o25 p27)(includes o25 p49)(includes o25 p60)(includes o25 p85)(includes o25 p93)(includes o25 p104)(includes o25 p142)(includes o25 p170)

(waiting o26)
(includes o26 p4)(includes o26 p10)(includes o26 p132)(includes o26 p145)

(waiting o27)
(includes o27 p20)

(waiting o28)
(includes o28 p32)(includes o28 p114)(includes o28 p133)(includes o28 p157)

(waiting o29)
(includes o29 p28)(includes o29 p61)(includes o29 p85)(includes o29 p122)

(waiting o30)
(includes o30 p57)(includes o30 p123)

(waiting o31)
(includes o31 p5)(includes o31 p22)(includes o31 p44)(includes o31 p67)(includes o31 p113)(includes o31 p136)

(waiting o32)
(includes o32 p42)(includes o32 p44)(includes o32 p74)(includes o32 p107)

(waiting o33)
(includes o33 p16)(includes o33 p53)(includes o33 p81)(includes o33 p138)(includes o33 p153)

(waiting o34)
(includes o34 p10)(includes o34 p17)(includes o34 p20)(includes o34 p26)(includes o34 p41)(includes o34 p54)(includes o34 p82)(includes o34 p104)(includes o34 p134)(includes o34 p139)

(waiting o35)
(includes o35 p7)(includes o35 p69)(includes o35 p77)(includes o35 p118)(includes o35 p170)

(waiting o36)
(includes o36 p26)(includes o36 p31)(includes o36 p104)(includes o36 p129)(includes o36 p138)

(waiting o37)
(includes o37 p13)(includes o37 p21)(includes o37 p28)(includes o37 p75)

(waiting o38)
(includes o38 p33)(includes o38 p49)(includes o38 p55)(includes o38 p125)(includes o38 p170)

(waiting o39)
(includes o39 p93)

(waiting o40)
(includes o40 p101)(includes o40 p121)(includes o40 p130)(includes o40 p154)

(waiting o41)
(includes o41 p11)(includes o41 p24)(includes o41 p84)(includes o41 p87)(includes o41 p101)(includes o41 p136)

(waiting o42)
(includes o42 p29)(includes o42 p36)(includes o42 p46)(includes o42 p115)(includes o42 p145)(includes o42 p149)(includes o42 p159)

(waiting o43)
(includes o43 p4)(includes o43 p17)(includes o43 p29)(includes o43 p33)(includes o43 p48)(includes o43 p89)(includes o43 p109)

(waiting o44)
(includes o44 p55)(includes o44 p115)

(waiting o45)
(includes o45 p112)

(waiting o46)
(includes o46 p7)(includes o46 p113)(includes o46 p157)

(waiting o47)
(includes o47 p18)(includes o47 p32)(includes o47 p37)(includes o47 p46)(includes o47 p64)(includes o47 p85)(includes o47 p91)

(waiting o48)
(includes o48 p58)(includes o48 p91)(includes o48 p122)

(waiting o49)
(includes o49 p3)(includes o49 p27)(includes o49 p33)(includes o49 p54)(includes o49 p146)(includes o49 p162)

(waiting o50)
(includes o50 p13)(includes o50 p68)(includes o50 p129)

(waiting o51)
(includes o51 p16)(includes o51 p18)(includes o51 p42)(includes o51 p66)(includes o51 p83)(includes o51 p90)(includes o51 p92)(includes o51 p103)(includes o51 p112)(includes o51 p113)(includes o51 p167)

(waiting o52)
(includes o52 p5)(includes o52 p99)(includes o52 p117)(includes o52 p145)

(waiting o53)
(includes o53 p14)(includes o53 p20)(includes o53 p70)(includes o53 p72)(includes o53 p84)(includes o53 p89)(includes o53 p126)(includes o53 p127)(includes o53 p142)

(waiting o54)
(includes o54 p14)(includes o54 p17)(includes o54 p35)(includes o54 p93)(includes o54 p110)(includes o54 p134)

(waiting o55)
(includes o55 p20)(includes o55 p42)(includes o55 p118)(includes o55 p126)(includes o55 p145)

(waiting o56)
(includes o56 p28)(includes o56 p50)(includes o56 p70)(includes o56 p101)

(waiting o57)
(includes o57 p5)(includes o57 p85)(includes o57 p147)

(waiting o58)
(includes o58 p62)(includes o58 p67)(includes o58 p132)(includes o58 p155)

(waiting o59)
(includes o59 p56)(includes o59 p88)(includes o59 p136)(includes o59 p156)

(waiting o60)
(includes o60 p51)(includes o60 p60)(includes o60 p78)(includes o60 p98)(includes o60 p104)

(waiting o61)
(includes o61 p6)(includes o61 p18)(includes o61 p24)(includes o61 p38)(includes o61 p41)(includes o61 p42)(includes o61 p60)(includes o61 p65)(includes o61 p66)

(waiting o62)
(includes o62 p10)(includes o62 p24)(includes o62 p28)(includes o62 p76)(includes o62 p98)

(waiting o63)
(includes o63 p58)(includes o63 p115)(includes o63 p121)(includes o63 p158)

(waiting o64)
(includes o64 p12)(includes o64 p21)(includes o64 p28)(includes o64 p36)(includes o64 p56)(includes o64 p61)(includes o64 p63)(includes o64 p132)(includes o64 p144)

(waiting o65)
(includes o65 p30)(includes o65 p38)

(waiting o66)
(includes o66 p10)(includes o66 p81)(includes o66 p96)(includes o66 p102)(includes o66 p109)(includes o66 p119)(includes o66 p134)(includes o66 p150)

(waiting o67)
(includes o67 p34)(includes o67 p36)(includes o67 p52)(includes o67 p84)(includes o67 p150)

(waiting o68)
(includes o68 p19)(includes o68 p23)(includes o68 p28)(includes o68 p34)(includes o68 p111)(includes o68 p133)

(waiting o69)
(includes o69 p22)(includes o69 p105)(includes o69 p161)

(waiting o70)
(includes o70 p14)(includes o70 p54)(includes o70 p71)(includes o70 p126)

(waiting o71)
(includes o71 p20)(includes o71 p39)(includes o71 p48)(includes o71 p84)(includes o71 p120)

(waiting o72)
(includes o72 p152)(includes o72 p153)

(waiting o73)
(includes o73 p83)(includes o73 p96)(includes o73 p105)(includes o73 p111)

(waiting o74)
(includes o74 p7)(includes o74 p35)(includes o74 p43)(includes o74 p117)(includes o74 p145)(includes o74 p159)

(waiting o75)
(includes o75 p9)(includes o75 p19)(includes o75 p55)(includes o75 p57)(includes o75 p111)(includes o75 p156)(includes o75 p167)

(waiting o76)
(includes o76 p63)(includes o76 p141)

(waiting o77)
(includes o77 p41)(includes o77 p61)(includes o77 p122)

(waiting o78)
(includes o78 p18)(includes o78 p42)(includes o78 p47)(includes o78 p59)(includes o78 p150)(includes o78 p152)

(waiting o79)
(includes o79 p15)(includes o79 p21)(includes o79 p44)(includes o79 p86)(includes o79 p96)(includes o79 p151)

(waiting o80)
(includes o80 p6)(includes o80 p8)(includes o80 p19)(includes o80 p30)(includes o80 p111)

(waiting o81)
(includes o81 p25)(includes o81 p86)(includes o81 p107)(includes o81 p111)(includes o81 p121)(includes o81 p124)(includes o81 p128)(includes o81 p137)(includes o81 p165)

(waiting o82)
(includes o82 p66)(includes o82 p111)(includes o82 p127)(includes o82 p130)(includes o82 p151)

(waiting o83)
(includes o83 p1)(includes o83 p6)(includes o83 p65)(includes o83 p106)(includes o83 p120)(includes o83 p134)(includes o83 p135)

(waiting o84)
(includes o84 p5)(includes o84 p16)(includes o84 p43)(includes o84 p64)(includes o84 p106)(includes o84 p140)(includes o84 p145)

(waiting o85)
(includes o85 p57)(includes o85 p94)(includes o85 p97)(includes o85 p156)

(waiting o86)
(includes o86 p23)(includes o86 p33)(includes o86 p107)(includes o86 p143)(includes o86 p147)

(waiting o87)
(includes o87 p11)(includes o87 p76)(includes o87 p90)(includes o87 p120)

(waiting o88)
(includes o88 p7)(includes o88 p25)(includes o88 p29)(includes o88 p91)

(waiting o89)
(includes o89 p38)(includes o89 p113)

(waiting o90)
(includes o90 p61)(includes o90 p133)(includes o90 p136)

(waiting o91)
(includes o91 p8)(includes o91 p66)(includes o91 p87)

(waiting o92)
(includes o92 p105)(includes o92 p109)(includes o92 p130)(includes o92 p141)(includes o92 p148)(includes o92 p154)

(waiting o93)
(includes o93 p54)(includes o93 p61)(includes o93 p106)

(waiting o94)
(includes o94 p70)(includes o94 p121)(includes o94 p161)

(waiting o95)
(includes o95 p18)(includes o95 p25)(includes o95 p32)(includes o95 p109)

(waiting o96)
(includes o96 p30)(includes o96 p32)(includes o96 p42)(includes o96 p53)(includes o96 p69)(includes o96 p85)(includes o96 p104)(includes o96 p115)(includes o96 p147)(includes o96 p159)

(waiting o97)
(includes o97 p90)(includes o97 p148)(includes o97 p161)

(waiting o98)
(includes o98 p45)(includes o98 p67)(includes o98 p150)(includes o98 p160)

(waiting o99)
(includes o99 p95)(includes o99 p164)

(waiting o100)
(includes o100 p3)(includes o100 p40)(includes o100 p61)(includes o100 p89)(includes o100 p105)(includes o100 p133)(includes o100 p139)(includes o100 p150)

(waiting o101)
(includes o101 p96)

(waiting o102)
(includes o102 p34)(includes o102 p128)(includes o102 p151)

(waiting o103)
(includes o103 p12)(includes o103 p26)(includes o103 p31)(includes o103 p39)(includes o103 p59)

(waiting o104)
(includes o104 p3)(includes o104 p151)

(waiting o105)
(includes o105 p32)(includes o105 p53)(includes o105 p70)(includes o105 p111)(includes o105 p131)(includes o105 p148)(includes o105 p157)

(waiting o106)
(includes o106 p24)(includes o106 p26)(includes o106 p31)(includes o106 p60)(includes o106 p68)(includes o106 p70)(includes o106 p150)(includes o106 p157)

(waiting o107)
(includes o107 p14)(includes o107 p31)(includes o107 p34)(includes o107 p56)(includes o107 p138)

(waiting o108)
(includes o108 p23)(includes o108 p51)(includes o108 p87)(includes o108 p90)

(waiting o109)
(includes o109 p9)(includes o109 p28)(includes o109 p40)(includes o109 p83)(includes o109 p110)

(waiting o110)
(includes o110 p37)(includes o110 p56)(includes o110 p68)(includes o110 p138)(includes o110 p155)

(waiting o111)
(includes o111 p83)(includes o111 p94)(includes o111 p141)

(waiting o112)
(includes o112 p22)(includes o112 p42)(includes o112 p57)(includes o112 p80)(includes o112 p119)(includes o112 p170)

(waiting o113)
(includes o113 p20)(includes o113 p43)(includes o113 p47)(includes o113 p59)(includes o113 p95)(includes o113 p97)(includes o113 p102)(includes o113 p122)(includes o113 p127)(includes o113 p158)

(waiting o114)
(includes o114 p78)(includes o114 p109)

(waiting o115)
(includes o115 p33)(includes o115 p37)(includes o115 p44)(includes o115 p58)(includes o115 p116)

(waiting o116)
(includes o116 p59)(includes o116 p93)(includes o116 p98)(includes o116 p117)(includes o116 p165)

(waiting o117)
(includes o117 p1)(includes o117 p12)(includes o117 p27)(includes o117 p38)(includes o117 p78)(includes o117 p85)(includes o117 p88)(includes o117 p126)(includes o117 p150)

(waiting o118)
(includes o118 p26)(includes o118 p112)(includes o118 p114)(includes o118 p118)(includes o118 p147)(includes o118 p149)(includes o118 p155)(includes o118 p169)

(waiting o119)
(includes o119 p4)(includes o119 p6)(includes o119 p12)(includes o119 p22)(includes o119 p25)(includes o119 p54)(includes o119 p142)

(waiting o120)
(includes o120 p6)(includes o120 p44)(includes o120 p48)(includes o120 p73)(includes o120 p142)

(waiting o121)
(includes o121 p5)(includes o121 p9)(includes o121 p75)(includes o121 p82)(includes o121 p95)

(waiting o122)
(includes o122 p42)(includes o122 p147)

(waiting o123)
(includes o123 p28)(includes o123 p88)(includes o123 p145)(includes o123 p146)(includes o123 p153)(includes o123 p156)

(waiting o124)
(includes o124 p23)(includes o124 p28)(includes o124 p63)(includes o124 p103)(includes o124 p155)(includes o124 p165)(includes o124 p169)

(waiting o125)
(includes o125 p88)(includes o125 p163)

(waiting o126)
(includes o126 p21)(includes o126 p158)

(waiting o127)
(includes o127 p26)(includes o127 p35)(includes o127 p119)(includes o127 p123)(includes o127 p170)

(waiting o128)
(includes o128 p41)(includes o128 p67)(includes o128 p71)(includes o128 p90)

(waiting o129)
(includes o129 p2)(includes o129 p27)(includes o129 p35)(includes o129 p48)(includes o129 p134)(includes o129 p160)

(waiting o130)
(includes o130 p4)(includes o130 p23)(includes o130 p38)(includes o130 p59)(includes o130 p76)(includes o130 p85)(includes o130 p94)(includes o130 p164)

(waiting o131)
(includes o131 p1)(includes o131 p25)(includes o131 p31)(includes o131 p106)(includes o131 p130)

(waiting o132)
(includes o132 p28)(includes o132 p41)(includes o132 p118)(includes o132 p124)(includes o132 p166)

(waiting o133)
(includes o133 p84)(includes o133 p91)(includes o133 p139)(includes o133 p167)(includes o133 p169)

(waiting o134)
(includes o134 p10)(includes o134 p102)(includes o134 p115)(includes o134 p145)(includes o134 p157)

(waiting o135)
(includes o135 p45)(includes o135 p65)(includes o135 p88)(includes o135 p105)(includes o135 p161)

(waiting o136)
(includes o136 p24)(includes o136 p57)(includes o136 p58)(includes o136 p95)(includes o136 p114)(includes o136 p117)(includes o136 p155)

(waiting o137)
(includes o137 p18)(includes o137 p54)(includes o137 p91)(includes o137 p109)

(waiting o138)
(includes o138 p24)(includes o138 p90)(includes o138 p102)(includes o138 p103)(includes o138 p113)(includes o138 p144)(includes o138 p155)

(waiting o139)
(includes o139 p62)(includes o139 p124)(includes o139 p151)(includes o139 p153)(includes o139 p168)

(waiting o140)
(includes o140 p26)(includes o140 p31)(includes o140 p43)(includes o140 p78)(includes o140 p112)

(waiting o141)
(includes o141 p38)(includes o141 p40)(includes o141 p65)(includes o141 p70)(includes o141 p96)(includes o141 p98)(includes o141 p127)

(waiting o142)
(includes o142 p17)(includes o142 p43)(includes o142 p62)(includes o142 p93)(includes o142 p106)

(waiting o143)
(includes o143 p9)(includes o143 p10)(includes o143 p27)(includes o143 p35)(includes o143 p53)(includes o143 p95)(includes o143 p123)(includes o143 p145)(includes o143 p153)

(waiting o144)
(includes o144 p2)(includes o144 p20)(includes o144 p69)(includes o144 p109)(includes o144 p111)

(waiting o145)
(includes o145 p57)(includes o145 p64)(includes o145 p123)(includes o145 p136)

(waiting o146)
(includes o146 p45)(includes o146 p88)(includes o146 p136)(includes o146 p138)(includes o146 p151)

(waiting o147)
(includes o147 p1)(includes o147 p8)(includes o147 p17)(includes o147 p31)(includes o147 p104)

(waiting o148)
(includes o148 p12)(includes o148 p24)(includes o148 p70)(includes o148 p102)(includes o148 p109)(includes o148 p113)(includes o148 p116)(includes o148 p124)(includes o148 p143)(includes o148 p147)(includes o148 p150)(includes o148 p170)

(waiting o149)
(includes o149 p43)(includes o149 p52)(includes o149 p120)(includes o149 p142)(includes o149 p164)

(waiting o150)
(includes o150 p101)

(waiting o151)
(includes o151 p6)(includes o151 p66)

(waiting o152)
(includes o152 p3)(includes o152 p27)(includes o152 p43)(includes o152 p67)(includes o152 p72)(includes o152 p108)(includes o152 p139)

(waiting o153)
(includes o153 p16)(includes o153 p53)(includes o153 p58)(includes o153 p70)(includes o153 p97)(includes o153 p146)(includes o153 p147)

(waiting o154)
(includes o154 p52)(includes o154 p122)(includes o154 p123)(includes o154 p166)

(waiting o155)
(includes o155 p112)(includes o155 p131)

(waiting o156)
(includes o156 p21)(includes o156 p55)(includes o156 p62)(includes o156 p87)(includes o156 p105)(includes o156 p125)(includes o156 p151)

(waiting o157)
(includes o157 p4)(includes o157 p47)(includes o157 p55)(includes o157 p92)(includes o157 p143)(includes o157 p146)(includes o157 p156)(includes o157 p165)

(waiting o158)
(includes o158 p29)(includes o158 p39)(includes o158 p104)

(waiting o159)
(includes o159 p51)(includes o159 p131)(includes o159 p146)(includes o159 p157)(includes o159 p161)

(waiting o160)
(includes o160 p2)(includes o160 p48)(includes o160 p62)(includes o160 p94)(includes o160 p103)(includes o160 p105)

(waiting o161)
(includes o161 p105)(includes o161 p151)(includes o161 p152)

(waiting o162)
(includes o162 p50)(includes o162 p51)(includes o162 p55)(includes o162 p64)(includes o162 p125)(includes o162 p131)

(waiting o163)
(includes o163 p89)(includes o163 p90)(includes o163 p155)(includes o163 p156)(includes o163 p166)

(waiting o164)
(includes o164 p29)(includes o164 p37)(includes o164 p46)(includes o164 p62)(includes o164 p77)(includes o164 p94)

(waiting o165)
(includes o165 p13)

(waiting o166)
(includes o166 p65)(includes o166 p85)(includes o166 p92)(includes o166 p165)

(waiting o167)
(includes o167 p76)(includes o167 p83)(includes o167 p96)(includes o167 p151)

(waiting o168)
(includes o168 p3)(includes o168 p26)(includes o168 p33)(includes o168 p107)

(waiting o169)
(includes o169 p19)(includes o169 p72)(includes o169 p79)(includes o169 p83)(includes o169 p90)

(waiting o170)
(includes o170 p27)(includes o170 p44)(includes o170 p55)(includes o170 p64)(includes o170 p74)(includes o170 p76)(includes o170 p111)(includes o170 p130)(includes o170 p150)

(not-made p1)
(not-made p2)
(not-made p3)
(not-made p4)
(not-made p5)
(not-made p6)
(not-made p7)
(not-made p8)
(not-made p9)
(not-made p10)
(not-made p11)
(not-made p12)
(not-made p13)
(not-made p14)
(not-made p15)
(not-made p16)
(not-made p17)
(not-made p18)
(not-made p19)
(not-made p20)
(not-made p21)
(not-made p22)
(not-made p23)
(not-made p24)
(not-made p25)
(not-made p26)
(not-made p27)
(not-made p28)
(not-made p29)
(not-made p30)
(not-made p31)
(not-made p32)
(not-made p33)
(not-made p34)
(not-made p35)
(not-made p36)
(not-made p37)
(not-made p38)
(not-made p39)
(not-made p40)
(not-made p41)
(not-made p42)
(not-made p43)
(not-made p44)
(not-made p45)
(not-made p46)
(not-made p47)
(not-made p48)
(not-made p49)
(not-made p50)
(not-made p51)
(not-made p52)
(not-made p53)
(not-made p54)
(not-made p55)
(not-made p56)
(not-made p57)
(not-made p58)
(not-made p59)
(not-made p60)
(not-made p61)
(not-made p62)
(not-made p63)
(not-made p64)
(not-made p65)
(not-made p66)
(not-made p67)
(not-made p68)
(not-made p69)
(not-made p70)
(not-made p71)
(not-made p72)
(not-made p73)
(not-made p74)
(not-made p75)
(not-made p76)
(not-made p77)
(not-made p78)
(not-made p79)
(not-made p80)
(not-made p81)
(not-made p82)
(not-made p83)
(not-made p84)
(not-made p85)
(not-made p86)
(not-made p87)
(not-made p88)
(not-made p89)
(not-made p90)
(not-made p91)
(not-made p92)
(not-made p93)
(not-made p94)
(not-made p95)
(not-made p96)
(not-made p97)
(not-made p98)
(not-made p99)
(not-made p100)
(not-made p101)
(not-made p102)
(not-made p103)
(not-made p104)
(not-made p105)
(not-made p106)
(not-made p107)
(not-made p108)
(not-made p109)
(not-made p110)
(not-made p111)
(not-made p112)
(not-made p113)
(not-made p114)
(not-made p115)
(not-made p116)
(not-made p117)
(not-made p118)
(not-made p119)
(not-made p120)
(not-made p121)
(not-made p122)
(not-made p123)
(not-made p124)
(not-made p125)
(not-made p126)
(not-made p127)
(not-made p128)
(not-made p129)
(not-made p130)
(not-made p131)
(not-made p132)
(not-made p133)
(not-made p134)
(not-made p135)
(not-made p136)
(not-made p137)
(not-made p138)
(not-made p139)
(not-made p140)
(not-made p141)
(not-made p142)
(not-made p143)
(not-made p144)
(not-made p145)
(not-made p146)
(not-made p147)
(not-made p148)
(not-made p149)
(not-made p150)
(not-made p151)
(not-made p152)
(not-made p153)
(not-made p154)
(not-made p155)
(not-made p156)
(not-made p157)
(not-made p158)
(not-made p159)
(not-made p160)
(not-made p161)
(not-made p162)
(not-made p163)
(not-made p164)
(not-made p165)
(not-made p166)
(not-made p167)
(not-made p168)
(not-made p169)
(not-made p170)

(= (total-cost) 0)

)

(:goal
(and
(shipped o1)
(shipped o2)
(shipped o3)
(shipped o4)
(shipped o5)
(shipped o6)
(shipped o7)
(shipped o8)
(shipped o9)
(shipped o10)
(shipped o11)
(shipped o12)
(shipped o13)
(shipped o14)
(shipped o15)
(shipped o16)
(shipped o17)
(shipped o18)
(shipped o19)
(shipped o20)
(shipped o21)
(shipped o22)
(shipped o23)
(shipped o24)
(shipped o25)
(shipped o26)
(shipped o27)
(shipped o28)
(shipped o29)
(shipped o30)
(shipped o31)
(shipped o32)
(shipped o33)
(shipped o34)
(shipped o35)
(shipped o36)
(shipped o37)
(shipped o38)
(shipped o39)
(shipped o40)
(shipped o41)
(shipped o42)
(shipped o43)
(shipped o44)
(shipped o45)
(shipped o46)
(shipped o47)
(shipped o48)
(shipped o49)
(shipped o50)
(shipped o51)
(shipped o52)
(shipped o53)
(shipped o54)
(shipped o55)
(shipped o56)
(shipped o57)
(shipped o58)
(shipped o59)
(shipped o60)
(shipped o61)
(shipped o62)
(shipped o63)
(shipped o64)
(shipped o65)
(shipped o66)
(shipped o67)
(shipped o68)
(shipped o69)
(shipped o70)
(shipped o71)
(shipped o72)
(shipped o73)
(shipped o74)
(shipped o75)
(shipped o76)
(shipped o77)
(shipped o78)
(shipped o79)
(shipped o80)
(shipped o81)
(shipped o82)
(shipped o83)
(shipped o84)
(shipped o85)
(shipped o86)
(shipped o87)
(shipped o88)
(shipped o89)
(shipped o90)
(shipped o91)
(shipped o92)
(shipped o93)
(shipped o94)
(shipped o95)
(shipped o96)
(shipped o97)
(shipped o98)
(shipped o99)
(shipped o100)
(shipped o101)
(shipped o102)
(shipped o103)
(shipped o104)
(shipped o105)
(shipped o106)
(shipped o107)
(shipped o108)
(shipped o109)
(shipped o110)
(shipped o111)
(shipped o112)
(shipped o113)
(shipped o114)
(shipped o115)
(shipped o116)
(shipped o117)
(shipped o118)
(shipped o119)
(shipped o120)
(shipped o121)
(shipped o122)
(shipped o123)
(shipped o124)
(shipped o125)
(shipped o126)
(shipped o127)
(shipped o128)
(shipped o129)
(shipped o130)
(shipped o131)
(shipped o132)
(shipped o133)
(shipped o134)
(shipped o135)
(shipped o136)
(shipped o137)
(shipped o138)
(shipped o139)
(shipped o140)
(shipped o141)
(shipped o142)
(shipped o143)
(shipped o144)
(shipped o145)
(shipped o146)
(shipped o147)
(shipped o148)
(shipped o149)
(shipped o150)
(shipped o151)
(shipped o152)
(shipped o153)
(shipped o154)
(shipped o155)
(shipped o156)
(shipped o157)
(shipped o158)
(shipped o159)
(shipped o160)
(shipped o161)
(shipped o162)
(shipped o163)
(shipped o164)
(shipped o165)
(shipped o166)
(shipped o167)
(shipped o168)
(shipped o169)
(shipped o170)
))

(:metric minimize (total-cost))

)

