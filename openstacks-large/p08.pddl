(define (problem os-sequencedstrips-p210_2)
(:domain openstacks-sequencedstrips-nonADL-nonNegated)
(:objects 
n0 n1 n2 n3 n4 n5 n6 n7 n8 n9 n10 n11 n12 n13 n14 n15 n16 n17 n18 n19 n20 n21 n22 n23 n24 n25 n26 n27 n28 n29 n30 n31 n32 n33 n34 n35 n36 n37 n38 n39 n40 n41 n42 n43 n44 n45 n46 n47 n48 n49 n50 n51 n52 n53 n54 n55 n56 n57 n58 n59 n60 n61 n62 n63 n64 n65 n66 n67 n68 n69 n70 n71 n72 n73 n74 n75 n76 n77 n78 n79 n80 n81 n82 n83 n84 n85 n86 n87 n88 n89 n90 n91 n92 n93 n94 n95 n96 n97 n98 n99 n100 n101 n102 n103 n104 n105 n106 n107 n108 n109 n110 n111 n112 n113 n114 n115 n116 n117 n118 n119 n120 n121 n122 n123 n124 n125 n126 n127 n128 n129 n130 n131 n132 n133 n134 n135 n136 n137 n138 n139 n140 n141 n142 n143 n144 n145 n146 n147 n148 n149 n150 n151 n152 n153 n154 n155 n156 n157 n158 n159 n160 n161 n162 n163 n164 n165 n166 n167 n168 n169 n170 n171 n172 n173 n174 n175 n176 n177 n178 n179 n180 n181 n182 n183 n184 n185 n186 n187 n188 n189 n190 n191 n192 n193 n194 n195 n196 n197 n198 n199 n200 n201 n202 n203 n204 n205 n206 n207 n208 n209 n210  - count
)

(:init
(next-count n0 n1) (next-count n1 n2) (next-count n2 n3) (next-count n3 n4) (next-count n4 n5) (next-count n5 n6) (next-count n6 n7) (next-count n7 n8) (next-count n8 n9) (next-count n9 n10) (next-count n10 n11) (next-count n11 n12) (next-count n12 n13) (next-count n13 n14) (next-count n14 n15) (next-count n15 n16) (next-count n16 n17) (next-count n17 n18) (next-count n18 n19) (next-count n19 n20) (next-count n20 n21) (next-count n21 n22) (next-count n22 n23) (next-count n23 n24) (next-count n24 n25) (next-count n25 n26) (next-count n26 n27) (next-count n27 n28) (next-count n28 n29) (next-count n29 n30) (next-count n30 n31) (next-count n31 n32) (next-count n32 n33) (next-count n33 n34) (next-count n34 n35) (next-count n35 n36) (next-count n36 n37) (next-count n37 n38) (next-count n38 n39) (next-count n39 n40) (next-count n40 n41) (next-count n41 n42) (next-count n42 n43) (next-count n43 n44) (next-count n44 n45) (next-count n45 n46) (next-count n46 n47) (next-count n47 n48) (next-count n48 n49) (next-count n49 n50) (next-count n50 n51) (next-count n51 n52) (next-count n52 n53) (next-count n53 n54) (next-count n54 n55) (next-count n55 n56) (next-count n56 n57) (next-count n57 n58) (next-count n58 n59) (next-count n59 n60) (next-count n60 n61) (next-count n61 n62) (next-count n62 n63) (next-count n63 n64) (next-count n64 n65) (next-count n65 n66) (next-count n66 n67) (next-count n67 n68) (next-count n68 n69) (next-count n69 n70) (next-count n70 n71) (next-count n71 n72) (next-count n72 n73) (next-count n73 n74) (next-count n74 n75) (next-count n75 n76) (next-count n76 n77) (next-count n77 n78) (next-count n78 n79) (next-count n79 n80) (next-count n80 n81) (next-count n81 n82) (next-count n82 n83) (next-count n83 n84) (next-count n84 n85) (next-count n85 n86) (next-count n86 n87) (next-count n87 n88) (next-count n88 n89) (next-count n89 n90) (next-count n90 n91) (next-count n91 n92) (next-count n92 n93) (next-count n93 n94) (next-count n94 n95) (next-count n95 n96) (next-count n96 n97) (next-count n97 n98) (next-count n98 n99) (next-count n99 n100) (next-count n100 n101) (next-count n101 n102) (next-count n102 n103) (next-count n103 n104) (next-count n104 n105) (next-count n105 n106) (next-count n106 n107) (next-count n107 n108) (next-count n108 n109) (next-count n109 n110) (next-count n110 n111) (next-count n111 n112) (next-count n112 n113) (next-count n113 n114) (next-count n114 n115) (next-count n115 n116) (next-count n116 n117) (next-count n117 n118) (next-count n118 n119) (next-count n119 n120) (next-count n120 n121) (next-count n121 n122) (next-count n122 n123) (next-count n123 n124) (next-count n124 n125) (next-count n125 n126) (next-count n126 n127) (next-count n127 n128) (next-count n128 n129) (next-count n129 n130) (next-count n130 n131) (next-count n131 n132) (next-count n132 n133) (next-count n133 n134) (next-count n134 n135) (next-count n135 n136) (next-count n136 n137) (next-count n137 n138) (next-count n138 n139) (next-count n139 n140) (next-count n140 n141) (next-count n141 n142) (next-count n142 n143) (next-count n143 n144) (next-count n144 n145) (next-count n145 n146) (next-count n146 n147) (next-count n147 n148) (next-count n148 n149) (next-count n149 n150) (next-count n150 n151) (next-count n151 n152) (next-count n152 n153) (next-count n153 n154) (next-count n154 n155) (next-count n155 n156) (next-count n156 n157) (next-count n157 n158) (next-count n158 n159) (next-count n159 n160) (next-count n160 n161) (next-count n161 n162) (next-count n162 n163) (next-count n163 n164) (next-count n164 n165) (next-count n165 n166) (next-count n166 n167) (next-count n167 n168) (next-count n168 n169) (next-count n169 n170) (next-count n170 n171) (next-count n171 n172) (next-count n172 n173) (next-count n173 n174) (next-count n174 n175) (next-count n175 n176) (next-count n176 n177) (next-count n177 n178) (next-count n178 n179) (next-count n179 n180) (next-count n180 n181) (next-count n181 n182) (next-count n182 n183) (next-count n183 n184) (next-count n184 n185) (next-count n185 n186) (next-count n186 n187) (next-count n187 n188) (next-count n188 n189) (next-count n189 n190) (next-count n190 n191) (next-count n191 n192) (next-count n192 n193) (next-count n193 n194) (next-count n194 n195) (next-count n195 n196) (next-count n196 n197) (next-count n197 n198) (next-count n198 n199) (next-count n199 n200) (next-count n200 n201) (next-count n201 n202) (next-count n202 n203) (next-count n203 n204) (next-count n204 n205) (next-count n205 n206) (next-count n206 n207) (next-count n207 n208) (next-count n208 n209) (next-count n209 n210) 
(stacks-avail n0)

(waiting o1)
(includes o1 p91)(includes o1 p112)(includes o1 p114)(includes o1 p134)(includes o1 p138)(includes o1 p161)(includes o1 p207)

(waiting o2)
(includes o2 p51)(includes o2 p188)(includes o2 p191)(includes o2 p202)

(waiting o3)
(includes o3 p9)(includes o3 p26)(includes o3 p29)(includes o3 p84)(includes o3 p138)(includes o3 p140)(includes o3 p156)(includes o3 p157)

(waiting o4)
(includes o4 p26)(includes o4 p54)(includes o4 p109)(includes o4 p141)(includes o4 p142)(includes o4 p175)

(waiting o5)
(includes o5 p18)(includes o5 p19)(includes o5 p52)(includes o5 p83)(includes o5 p132)

(waiting o6)
(includes o6 p61)(includes o6 p96)(includes o6 p108)(includes o6 p116)(includes o6 p119)(includes o6 p159)

(waiting o7)
(includes o7 p60)(includes o7 p61)(includes o7 p62)(includes o7 p119)(includes o7 p127)(includes o7 p155)(includes o7 p183)

(waiting o8)
(includes o8 p89)(includes o8 p121)(includes o8 p145)

(waiting o9)
(includes o9 p90)(includes o9 p103)(includes o9 p148)(includes o9 p177)(includes o9 p185)

(waiting o10)
(includes o10 p53)

(waiting o11)
(includes o11 p33)(includes o11 p63)(includes o11 p101)(includes o11 p126)(includes o11 p152)(includes o11 p156)(includes o11 p201)

(waiting o12)
(includes o12 p75)(includes o12 p87)(includes o12 p207)

(waiting o13)
(includes o13 p25)(includes o13 p114)(includes o13 p141)(includes o13 p147)(includes o13 p190)

(waiting o14)
(includes o14 p18)(includes o14 p41)(includes o14 p45)(includes o14 p87)(includes o14 p96)(includes o14 p167)(includes o14 p189)

(waiting o15)
(includes o15 p78)(includes o15 p122)

(waiting o16)
(includes o16 p2)(includes o16 p18)(includes o16 p32)(includes o16 p88)(includes o16 p127)(includes o16 p189)

(waiting o17)
(includes o17 p59)(includes o17 p84)(includes o17 p86)(includes o17 p163)

(waiting o18)
(includes o18 p4)(includes o18 p77)(includes o18 p130)

(waiting o19)
(includes o19 p20)(includes o19 p54)(includes o19 p85)(includes o19 p94)(includes o19 p103)(includes o19 p184)(includes o19 p191)

(waiting o20)
(includes o20 p23)(includes o20 p26)(includes o20 p53)(includes o20 p67)(includes o20 p111)(includes o20 p124)(includes o20 p128)(includes o20 p150)

(waiting o21)
(includes o21 p6)(includes o21 p25)(includes o21 p87)(includes o21 p117)(includes o21 p121)(includes o21 p192)

(waiting o22)
(includes o22 p5)(includes o22 p98)(includes o22 p120)(includes o22 p178)(includes o22 p179)(includes o22 p191)

(waiting o23)
(includes o23 p68)(includes o23 p72)(includes o23 p86)

(waiting o24)
(includes o24 p7)(includes o24 p86)(includes o24 p123)(includes o24 p134)(includes o24 p138)(includes o24 p165)(includes o24 p193)

(waiting o25)
(includes o25 p16)(includes o25 p50)(includes o25 p107)(includes o25 p121)(includes o25 p131)

(waiting o26)
(includes o26 p83)(includes o26 p93)(includes o26 p107)(includes o26 p122)(includes o26 p142)(includes o26 p143)(includes o26 p182)(includes o26 p192)

(waiting o27)
(includes o27 p9)(includes o27 p51)(includes o27 p120)(includes o27 p129)(includes o27 p131)(includes o27 p161)(includes o27 p163)(includes o27 p185)

(waiting o28)
(includes o28 p6)(includes o28 p123)(includes o28 p178)

(waiting o29)
(includes o29 p77)(includes o29 p169)(includes o29 p195)(includes o29 p206)(includes o29 p207)

(waiting o30)
(includes o30 p3)(includes o30 p7)(includes o30 p12)(includes o30 p15)(includes o30 p46)(includes o30 p49)(includes o30 p180)(includes o30 p192)

(waiting o31)
(includes o31 p19)(includes o31 p21)(includes o31 p23)

(waiting o32)
(includes o32 p22)(includes o32 p36)(includes o32 p41)(includes o32 p64)(includes o32 p89)(includes o32 p130)(includes o32 p147)(includes o32 p158)(includes o32 p159)(includes o32 p199)

(waiting o33)
(includes o33 p87)(includes o33 p117)(includes o33 p132)

(waiting o34)
(includes o34 p13)(includes o34 p25)(includes o34 p63)(includes o34 p79)(includes o34 p95)(includes o34 p151)(includes o34 p188)

(waiting o35)
(includes o35 p3)(includes o35 p7)(includes o35 p14)(includes o35 p122)(includes o35 p145)

(waiting o36)
(includes o36 p23)(includes o36 p76)(includes o36 p139)(includes o36 p178)(includes o36 p185)(includes o36 p192)

(waiting o37)
(includes o37 p7)(includes o37 p25)(includes o37 p42)(includes o37 p86)(includes o37 p140)(includes o37 p181)(includes o37 p191)(includes o37 p194)

(waiting o38)
(includes o38 p20)(includes o38 p51)(includes o38 p98)(includes o38 p161)

(waiting o39)
(includes o39 p15)(includes o39 p49)(includes o39 p78)(includes o39 p135)

(waiting o40)
(includes o40 p7)(includes o40 p10)(includes o40 p21)(includes o40 p36)(includes o40 p55)(includes o40 p61)(includes o40 p62)(includes o40 p120)(includes o40 p134)

(waiting o41)
(includes o41 p38)(includes o41 p62)(includes o41 p78)(includes o41 p127)(includes o41 p147)(includes o41 p153)(includes o41 p173)(includes o41 p186)(includes o41 p197)

(waiting o42)
(includes o42 p32)(includes o42 p112)(includes o42 p129)(includes o42 p199)

(waiting o43)
(includes o43 p9)(includes o43 p101)

(waiting o44)
(includes o44 p132)(includes o44 p162)(includes o44 p167)(includes o44 p172)

(waiting o45)
(includes o45 p29)(includes o45 p77)(includes o45 p101)(includes o45 p147)(includes o45 p171)(includes o45 p191)(includes o45 p194)

(waiting o46)
(includes o46 p18)(includes o46 p79)(includes o46 p83)(includes o46 p103)(includes o46 p163)(includes o46 p177)(includes o46 p178)

(waiting o47)
(includes o47 p10)

(waiting o48)
(includes o48 p14)(includes o48 p53)(includes o48 p77)(includes o48 p82)(includes o48 p87)(includes o48 p96)(includes o48 p185)

(waiting o49)
(includes o49 p3)(includes o49 p7)(includes o49 p77)(includes o49 p110)(includes o49 p142)(includes o49 p192)(includes o49 p199)

(waiting o50)
(includes o50 p4)(includes o50 p49)(includes o50 p64)(includes o50 p67)(includes o50 p92)(includes o50 p101)(includes o50 p117)(includes o50 p153)(includes o50 p209)

(waiting o51)
(includes o51 p35)(includes o51 p47)(includes o51 p60)(includes o51 p63)(includes o51 p70)(includes o51 p80)(includes o51 p120)(includes o51 p122)(includes o51 p161)(includes o51 p181)(includes o51 p188)

(waiting o52)
(includes o52 p75)(includes o52 p105)(includes o52 p109)

(waiting o53)
(includes o53 p52)(includes o53 p59)(includes o53 p73)(includes o53 p112)(includes o53 p147)(includes o53 p176)

(waiting o54)
(includes o54 p39)(includes o54 p93)(includes o54 p132)(includes o54 p169)(includes o54 p189)(includes o54 p209)

(waiting o55)
(includes o55 p26)(includes o55 p71)(includes o55 p80)(includes o55 p152)(includes o55 p173)(includes o55 p208)

(waiting o56)
(includes o56 p44)(includes o56 p48)(includes o56 p81)(includes o56 p92)(includes o56 p139)(includes o56 p184)(includes o56 p199)

(waiting o57)
(includes o57 p19)(includes o57 p27)(includes o57 p29)(includes o57 p56)(includes o57 p74)(includes o57 p99)(includes o57 p108)(includes o57 p113)(includes o57 p139)(includes o57 p166)(includes o57 p206)

(waiting o58)
(includes o58 p1)(includes o58 p18)(includes o58 p32)(includes o58 p60)(includes o58 p98)(includes o58 p137)

(waiting o59)
(includes o59 p7)(includes o59 p48)(includes o59 p135)(includes o59 p154)

(waiting o60)
(includes o60 p8)(includes o60 p49)(includes o60 p88)(includes o60 p180)

(waiting o61)
(includes o61 p21)(includes o61 p42)(includes o61 p74)(includes o61 p195)(includes o61 p207)

(waiting o62)
(includes o62 p43)(includes o62 p72)(includes o62 p80)(includes o62 p89)(includes o62 p135)(includes o62 p151)(includes o62 p197)

(waiting o63)
(includes o63 p59)(includes o63 p166)

(waiting o64)
(includes o64 p37)(includes o64 p74)(includes o64 p83)(includes o64 p122)(includes o64 p140)(includes o64 p208)

(waiting o65)
(includes o65 p53)(includes o65 p72)(includes o65 p111)(includes o65 p126)(includes o65 p169)(includes o65 p197)

(waiting o66)
(includes o66 p10)(includes o66 p29)(includes o66 p31)(includes o66 p56)(includes o66 p142)(includes o66 p167)

(waiting o67)
(includes o67 p56)(includes o67 p67)(includes o67 p86)(includes o67 p166)(includes o67 p167)(includes o67 p178)(includes o67 p179)(includes o67 p184)

(waiting o68)
(includes o68 p43)(includes o68 p52)(includes o68 p70)(includes o68 p81)(includes o68 p109)(includes o68 p155)

(waiting o69)
(includes o69 p130)(includes o69 p157)

(waiting o70)
(includes o70 p58)(includes o70 p63)(includes o70 p74)(includes o70 p111)(includes o70 p126)(includes o70 p129)(includes o70 p146)(includes o70 p164)(includes o70 p166)(includes o70 p184)(includes o70 p201)

(waiting o71)
(includes o71 p3)(includes o71 p4)(includes o71 p66)(includes o71 p68)(includes o71 p76)(includes o71 p91)(includes o71 p101)(includes o71 p128)(includes o71 p164)(includes o71 p185)

(waiting o72)
(includes o72 p142)(includes o72 p144)(includes o72 p168)

(waiting o73)
(includes o73 p9)(includes o73 p33)(includes o73 p57)(includes o73 p151)(includes o73 p168)(includes o73 p180)(includes o73 p192)(includes o73 p196)

(waiting o74)
(includes o74 p17)(includes o74 p20)(includes o74 p36)(includes o74 p41)(includes o74 p64)(includes o74 p81)(includes o74 p85)(includes o74 p94)(includes o74 p108)(includes o74 p123)(includes o74 p142)(includes o74 p143)(includes o74 p151)(includes o74 p161)(includes o74 p164)(includes o74 p185)(includes o74 p201)

(waiting o75)
(includes o75 p101)(includes o75 p186)(includes o75 p197)

(waiting o76)
(includes o76 p15)(includes o76 p36)(includes o76 p49)(includes o76 p67)(includes o76 p86)(includes o76 p113)(includes o76 p178)(includes o76 p181)(includes o76 p184)(includes o76 p200)

(waiting o77)
(includes o77 p40)(includes o77 p72)(includes o77 p106)(includes o77 p116)(includes o77 p126)(includes o77 p163)(includes o77 p183)

(waiting o78)
(includes o78 p1)(includes o78 p30)(includes o78 p76)(includes o78 p91)

(waiting o79)
(includes o79 p12)(includes o79 p27)(includes o79 p46)(includes o79 p94)(includes o79 p157)(includes o79 p189)(includes o79 p190)(includes o79 p206)

(waiting o80)
(includes o80 p67)(includes o80 p80)(includes o80 p107)(includes o80 p130)(includes o80 p153)

(waiting o81)
(includes o81 p117)

(waiting o82)
(includes o82 p29)(includes o82 p52)(includes o82 p93)(includes o82 p176)(includes o82 p189)(includes o82 p193)

(waiting o83)
(includes o83 p79)(includes o83 p81)(includes o83 p111)(includes o83 p208)

(waiting o84)
(includes o84 p57)(includes o84 p87)(includes o84 p89)(includes o84 p111)(includes o84 p158)(includes o84 p164)(includes o84 p165)(includes o84 p199)

(waiting o85)
(includes o85 p38)(includes o85 p39)(includes o85 p75)(includes o85 p81)(includes o85 p105)(includes o85 p126)(includes o85 p143)(includes o85 p190)

(waiting o86)
(includes o86 p39)(includes o86 p97)(includes o86 p129)(includes o86 p153)(includes o86 p158)(includes o86 p186)(includes o86 p205)

(waiting o87)
(includes o87 p45)(includes o87 p61)(includes o87 p101)(includes o87 p109)(includes o87 p132)(includes o87 p149)(includes o87 p169)

(waiting o88)
(includes o88 p52)(includes o88 p75)(includes o88 p96)(includes o88 p130)(includes o88 p166)(includes o88 p167)

(waiting o89)
(includes o89 p4)(includes o89 p80)(includes o89 p128)(includes o89 p129)(includes o89 p167)

(waiting o90)
(includes o90 p151)(includes o90 p160)

(waiting o91)
(includes o91 p52)(includes o91 p104)(includes o91 p110)(includes o91 p141)(includes o91 p143)(includes o91 p176)(includes o91 p182)(includes o91 p205)

(waiting o92)
(includes o92 p53)(includes o92 p73)

(waiting o93)
(includes o93 p105)(includes o93 p163)(includes o93 p202)

(waiting o94)
(includes o94 p1)(includes o94 p31)(includes o94 p62)(includes o94 p77)(includes o94 p144)

(waiting o95)
(includes o95 p25)(includes o95 p67)(includes o95 p71)(includes o95 p85)(includes o95 p90)(includes o95 p134)(includes o95 p204)(includes o95 p209)

(waiting o96)
(includes o96 p2)(includes o96 p20)(includes o96 p29)(includes o96 p59)(includes o96 p62)(includes o96 p78)(includes o96 p101)(includes o96 p112)(includes o96 p145)(includes o96 p146)

(waiting o97)
(includes o97 p67)(includes o97 p193)

(waiting o98)
(includes o98 p13)(includes o98 p61)(includes o98 p76)(includes o98 p196)(includes o98 p199)

(waiting o99)
(includes o99 p17)(includes o99 p49)(includes o99 p114)(includes o99 p127)

(waiting o100)
(includes o100 p9)(includes o100 p27)(includes o100 p41)(includes o100 p80)(includes o100 p89)(includes o100 p146)(includes o100 p171)

(waiting o101)
(includes o101 p11)(includes o101 p36)(includes o101 p55)(includes o101 p59)(includes o101 p101)(includes o101 p156)(includes o101 p159)(includes o101 p209)

(waiting o102)
(includes o102 p6)(includes o102 p26)(includes o102 p30)(includes o102 p50)(includes o102 p65)(includes o102 p186)(includes o102 p187)(includes o102 p207)

(waiting o103)
(includes o103 p4)(includes o103 p7)(includes o103 p14)(includes o103 p55)(includes o103 p58)(includes o103 p91)(includes o103 p138)(includes o103 p152)(includes o103 p192)

(waiting o104)
(includes o104 p7)(includes o104 p31)(includes o104 p35)(includes o104 p42)(includes o104 p85)(includes o104 p88)(includes o104 p93)(includes o104 p144)(includes o104 p191)(includes o104 p203)(includes o104 p210)

(waiting o105)
(includes o105 p15)(includes o105 p25)(includes o105 p58)(includes o105 p90)(includes o105 p135)(includes o105 p142)(includes o105 p144)(includes o105 p169)

(waiting o106)
(includes o106 p6)(includes o106 p38)(includes o106 p97)(includes o106 p117)(includes o106 p147)(includes o106 p152)(includes o106 p155)(includes o106 p176)(includes o106 p187)

(waiting o107)
(includes o107 p34)(includes o107 p57)(includes o107 p74)(includes o107 p75)(includes o107 p86)(includes o107 p117)(includes o107 p141)(includes o107 p198)(includes o107 p208)

(waiting o108)
(includes o108 p29)(includes o108 p167)(includes o108 p195)

(waiting o109)
(includes o109 p113)(includes o109 p182)

(waiting o110)
(includes o110 p3)(includes o110 p6)(includes o110 p7)(includes o110 p13)(includes o110 p30)(includes o110 p95)(includes o110 p115)(includes o110 p127)(includes o110 p188)

(waiting o111)
(includes o111 p32)(includes o111 p69)(includes o111 p70)(includes o111 p98)(includes o111 p104)(includes o111 p173)(includes o111 p191)

(waiting o112)
(includes o112 p1)(includes o112 p23)(includes o112 p145)(includes o112 p162)(includes o112 p194)(includes o112 p202)

(waiting o113)
(includes o113 p35)(includes o113 p134)(includes o113 p147)(includes o113 p165)(includes o113 p190)(includes o113 p201)

(waiting o114)
(includes o114 p29)(includes o114 p33)(includes o114 p36)(includes o114 p81)(includes o114 p115)(includes o114 p122)(includes o114 p124)(includes o114 p128)(includes o114 p190)

(waiting o115)
(includes o115 p61)(includes o115 p107)(includes o115 p127)(includes o115 p194)

(waiting o116)
(includes o116 p17)(includes o116 p42)(includes o116 p81)(includes o116 p89)(includes o116 p147)(includes o116 p190)(includes o116 p197)(includes o116 p208)

(waiting o117)
(includes o117 p3)(includes o117 p40)(includes o117 p88)(includes o117 p115)(includes o117 p127)(includes o117 p174)

(waiting o118)
(includes o118 p27)(includes o118 p39)(includes o118 p68)(includes o118 p75)(includes o118 p89)(includes o118 p130)(includes o118 p164)(includes o118 p168)(includes o118 p189)(includes o118 p191)

(waiting o119)
(includes o119 p33)(includes o119 p102)(includes o119 p116)(includes o119 p148)(includes o119 p201)(includes o119 p202)

(waiting o120)
(includes o120 p70)(includes o120 p76)(includes o120 p124)

(waiting o121)
(includes o121 p52)(includes o121 p183)(includes o121 p184)

(waiting o122)
(includes o122 p25)(includes o122 p101)(includes o122 p195)(includes o122 p201)

(waiting o123)
(includes o123 p10)(includes o123 p13)(includes o123 p138)(includes o123 p192)

(waiting o124)
(includes o124 p73)(includes o124 p167)

(waiting o125)
(includes o125 p136)(includes o125 p183)

(waiting o126)
(includes o126 p9)(includes o126 p12)(includes o126 p21)(includes o126 p59)(includes o126 p71)(includes o126 p111)(includes o126 p150)

(waiting o127)
(includes o127 p5)(includes o127 p30)(includes o127 p40)(includes o127 p86)(includes o127 p107)(includes o127 p110)(includes o127 p115)(includes o127 p124)(includes o127 p138)(includes o127 p168)

(waiting o128)
(includes o128 p79)(includes o128 p167)(includes o128 p181)(includes o128 p185)

(waiting o129)
(includes o129 p20)(includes o129 p76)(includes o129 p107)(includes o129 p118)(includes o129 p150)(includes o129 p208)

(waiting o130)
(includes o130 p5)(includes o130 p25)(includes o130 p47)(includes o130 p51)(includes o130 p83)(includes o130 p84)(includes o130 p129)(includes o130 p155)(includes o130 p197)

(waiting o131)
(includes o131 p22)(includes o131 p23)(includes o131 p39)(includes o131 p110)(includes o131 p123)(includes o131 p142)

(waiting o132)
(includes o132 p20)(includes o132 p28)(includes o132 p47)(includes o132 p91)(includes o132 p106)(includes o132 p180)

(waiting o133)
(includes o133 p1)(includes o133 p2)(includes o133 p68)(includes o133 p83)(includes o133 p92)(includes o133 p95)(includes o133 p124)

(waiting o134)
(includes o134 p55)(includes o134 p63)(includes o134 p64)(includes o134 p192)

(waiting o135)
(includes o135 p11)(includes o135 p39)(includes o135 p64)(includes o135 p72)(includes o135 p198)

(waiting o136)
(includes o136 p85)(includes o136 p122)(includes o136 p202)

(waiting o137)
(includes o137 p16)(includes o137 p102)(includes o137 p136)

(waiting o138)
(includes o138 p3)(includes o138 p73)(includes o138 p96)(includes o138 p128)(includes o138 p142)(includes o138 p147)(includes o138 p156)(includes o138 p181)

(waiting o139)
(includes o139 p2)(includes o139 p85)(includes o139 p99)(includes o139 p119)(includes o139 p125)(includes o139 p134)(includes o139 p163)(includes o139 p181)(includes o139 p188)(includes o139 p201)(includes o139 p202)

(waiting o140)
(includes o140 p8)(includes o140 p24)(includes o140 p31)(includes o140 p74)(includes o140 p100)(includes o140 p204)

(waiting o141)
(includes o141 p6)(includes o141 p25)(includes o141 p61)(includes o141 p73)(includes o141 p107)(includes o141 p121)(includes o141 p178)(includes o141 p194)(includes o141 p207)

(waiting o142)
(includes o142 p77)(includes o142 p86)(includes o142 p92)(includes o142 p103)(includes o142 p118)(includes o142 p119)

(waiting o143)
(includes o143 p15)(includes o143 p128)(includes o143 p146)

(waiting o144)
(includes o144 p38)(includes o144 p87)(includes o144 p117)(includes o144 p157)

(waiting o145)
(includes o145 p18)(includes o145 p45)(includes o145 p105)(includes o145 p186)

(waiting o146)
(includes o146 p33)(includes o146 p46)(includes o146 p48)(includes o146 p76)(includes o146 p84)(includes o146 p119)(includes o146 p160)(includes o146 p170)(includes o146 p178)(includes o146 p201)

(waiting o147)
(includes o147 p18)(includes o147 p44)(includes o147 p82)(includes o147 p155)(includes o147 p166)

(waiting o148)
(includes o148 p9)(includes o148 p48)(includes o148 p67)(includes o148 p94)(includes o148 p155)(includes o148 p186)(includes o148 p197)

(waiting o149)
(includes o149 p15)(includes o149 p68)(includes o149 p69)(includes o149 p72)(includes o149 p87)(includes o149 p99)(includes o149 p196)

(waiting o150)
(includes o150 p34)(includes o150 p104)(includes o150 p105)(includes o150 p207)

(waiting o151)
(includes o151 p3)(includes o151 p9)(includes o151 p58)(includes o151 p59)(includes o151 p108)(includes o151 p126)(includes o151 p165)(includes o151 p189)

(waiting o152)
(includes o152 p2)(includes o152 p58)(includes o152 p134)(includes o152 p136)(includes o152 p161)(includes o152 p180)(includes o152 p182)(includes o152 p183)

(waiting o153)
(includes o153 p5)(includes o153 p48)(includes o153 p83)(includes o153 p99)(includes o153 p201)(includes o153 p202)

(waiting o154)
(includes o154 p19)(includes o154 p148)(includes o154 p159)(includes o154 p204)

(waiting o155)
(includes o155 p39)(includes o155 p105)(includes o155 p141)(includes o155 p203)(includes o155 p208)

(waiting o156)
(includes o156 p24)(includes o156 p33)(includes o156 p107)(includes o156 p137)(includes o156 p181)(includes o156 p185)

(waiting o157)
(includes o157 p118)(includes o157 p147)

(waiting o158)
(includes o158 p21)(includes o158 p105)

(waiting o159)
(includes o159 p36)(includes o159 p68)(includes o159 p171)(includes o159 p176)(includes o159 p196)(includes o159 p200)(includes o159 p202)

(waiting o160)
(includes o160 p11)(includes o160 p24)(includes o160 p26)(includes o160 p177)(includes o160 p197)

(waiting o161)
(includes o161 p101)(includes o161 p159)(includes o161 p189)

(waiting o162)
(includes o162 p3)(includes o162 p41)(includes o162 p103)(includes o162 p128)

(waiting o163)
(includes o163 p13)(includes o163 p30)(includes o163 p74)

(waiting o164)
(includes o164 p57)(includes o164 p99)(includes o164 p103)(includes o164 p196)(includes o164 p200)

(waiting o165)
(includes o165 p9)(includes o165 p54)(includes o165 p80)(includes o165 p90)(includes o165 p129)(includes o165 p137)(includes o165 p143)(includes o165 p157)(includes o165 p159)

(waiting o166)
(includes o166 p43)(includes o166 p52)(includes o166 p166)(includes o166 p170)

(waiting o167)
(includes o167 p1)(includes o167 p19)(includes o167 p78)(includes o167 p127)(includes o167 p138)(includes o167 p194)(includes o167 p196)

(waiting o168)
(includes o168 p64)(includes o168 p126)(includes o168 p171)

(waiting o169)
(includes o169 p55)(includes o169 p78)(includes o169 p103)(includes o169 p139)(includes o169 p201)

(waiting o170)
(includes o170 p1)(includes o170 p35)(includes o170 p91)(includes o170 p94)(includes o170 p168)

(waiting o171)
(includes o171 p77)(includes o171 p93)(includes o171 p114)(includes o171 p136)(includes o171 p160)(includes o171 p183)

(waiting o172)
(includes o172 p51)(includes o172 p124)(includes o172 p150)(includes o172 p155)(includes o172 p174)(includes o172 p192)

(waiting o173)
(includes o173 p35)(includes o173 p76)(includes o173 p95)(includes o173 p157)

(waiting o174)
(includes o174 p7)(includes o174 p33)(includes o174 p91)(includes o174 p115)(includes o174 p130)(includes o174 p180)(includes o174 p184)(includes o174 p200)

(waiting o175)
(includes o175 p2)(includes o175 p33)(includes o175 p66)(includes o175 p75)(includes o175 p84)(includes o175 p87)(includes o175 p128)

(waiting o176)
(includes o176 p48)(includes o176 p52)(includes o176 p111)(includes o176 p128)(includes o176 p142)(includes o176 p162)(includes o176 p182)(includes o176 p183)(includes o176 p196)

(waiting o177)
(includes o177 p7)(includes o177 p9)(includes o177 p60)(includes o177 p118)(includes o177 p133)(includes o177 p162)(includes o177 p165)

(waiting o178)
(includes o178 p35)(includes o178 p95)(includes o178 p106)(includes o178 p119)(includes o178 p130)(includes o178 p184)(includes o178 p201)

(waiting o179)
(includes o179 p93)(includes o179 p124)(includes o179 p126)(includes o179 p150)(includes o179 p152)(includes o179 p170)

(waiting o180)
(includes o180 p24)(includes o180 p82)

(waiting o181)
(includes o181 p30)(includes o181 p110)(includes o181 p117)

(waiting o182)
(includes o182 p7)(includes o182 p32)(includes o182 p159)

(waiting o183)
(includes o183 p48)(includes o183 p50)(includes o183 p85)(includes o183 p112)(includes o183 p123)(includes o183 p158)(includes o183 p159)(includes o183 p194)

(waiting o184)
(includes o184 p34)(includes o184 p58)(includes o184 p82)(includes o184 p185)

(waiting o185)
(includes o185 p26)(includes o185 p130)(includes o185 p139)

(waiting o186)
(includes o186 p1)(includes o186 p51)(includes o186 p125)(includes o186 p154)(includes o186 p180)

(waiting o187)
(includes o187 p14)(includes o187 p23)(includes o187 p28)(includes o187 p174)(includes o187 p192)

(waiting o188)
(includes o188 p68)(includes o188 p157)(includes o188 p162)(includes o188 p176)

(waiting o189)
(includes o189 p27)(includes o189 p30)(includes o189 p32)(includes o189 p56)(includes o189 p62)(includes o189 p81)(includes o189 p86)(includes o189 p164)(includes o189 p185)(includes o189 p194)

(waiting o190)
(includes o190 p13)(includes o190 p14)(includes o190 p57)(includes o190 p60)(includes o190 p105)

(waiting o191)
(includes o191 p27)(includes o191 p62)(includes o191 p63)(includes o191 p95)(includes o191 p131)(includes o191 p157)(includes o191 p175)

(waiting o192)
(includes o192 p9)(includes o192 p15)(includes o192 p49)(includes o192 p55)(includes o192 p151)

(waiting o193)
(includes o193 p29)(includes o193 p160)

(waiting o194)
(includes o194 p62)(includes o194 p140)

(waiting o195)
(includes o195 p23)(includes o195 p31)(includes o195 p93)(includes o195 p114)(includes o195 p187)(includes o195 p190)(includes o195 p197)

(waiting o196)
(includes o196 p1)(includes o196 p51)(includes o196 p118)(includes o196 p120)(includes o196 p143)(includes o196 p182)

(waiting o197)
(includes o197 p14)(includes o197 p24)(includes o197 p37)(includes o197 p65)(includes o197 p136)(includes o197 p161)(includes o197 p208)

(waiting o198)
(includes o198 p20)(includes o198 p23)(includes o198 p34)(includes o198 p44)(includes o198 p101)(includes o198 p137)(includes o198 p140)(includes o198 p154)

(waiting o199)
(includes o199 p9)(includes o199 p27)(includes o199 p73)(includes o199 p104)(includes o199 p117)(includes o199 p158)

(waiting o200)
(includes o200 p10)(includes o200 p32)(includes o200 p145)(includes o200 p174)(includes o200 p191)

(waiting o201)
(includes o201 p68)(includes o201 p86)(includes o201 p90)(includes o201 p138)(includes o201 p153)(includes o201 p167)(includes o201 p206)

(waiting o202)
(includes o202 p7)(includes o202 p55)(includes o202 p68)(includes o202 p115)(includes o202 p181)(includes o202 p186)(includes o202 p191)

(waiting o203)
(includes o203 p19)(includes o203 p74)(includes o203 p88)(includes o203 p102)(includes o203 p137)(includes o203 p185)

(waiting o204)
(includes o204 p10)(includes o204 p27)(includes o204 p37)(includes o204 p41)(includes o204 p57)(includes o204 p72)(includes o204 p102)(includes o204 p114)(includes o204 p178)(includes o204 p190)

(waiting o205)
(includes o205 p3)(includes o205 p4)(includes o205 p12)(includes o205 p16)(includes o205 p27)(includes o205 p45)(includes o205 p50)(includes o205 p91)(includes o205 p189)

(waiting o206)
(includes o206 p11)(includes o206 p72)(includes o206 p97)(includes o206 p126)(includes o206 p147)(includes o206 p203)(includes o206 p208)

(waiting o207)
(includes o207 p53)(includes o207 p72)(includes o207 p93)(includes o207 p126)(includes o207 p140)

(waiting o208)
(includes o208 p4)(includes o208 p7)(includes o208 p47)(includes o208 p158)(includes o208 p208)

(waiting o209)
(includes o209 p49)(includes o209 p56)(includes o209 p73)(includes o209 p76)(includes o209 p180)

(waiting o210)
(includes o210 p60)(includes o210 p72)(includes o210 p82)(includes o210 p83)(includes o210 p86)(includes o210 p96)(includes o210 p107)(includes o210 p157)

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
(not-made p171)
(not-made p172)
(not-made p173)
(not-made p174)
(not-made p175)
(not-made p176)
(not-made p177)
(not-made p178)
(not-made p179)
(not-made p180)
(not-made p181)
(not-made p182)
(not-made p183)
(not-made p184)
(not-made p185)
(not-made p186)
(not-made p187)
(not-made p188)
(not-made p189)
(not-made p190)
(not-made p191)
(not-made p192)
(not-made p193)
(not-made p194)
(not-made p195)
(not-made p196)
(not-made p197)
(not-made p198)
(not-made p199)
(not-made p200)
(not-made p201)
(not-made p202)
(not-made p203)
(not-made p204)
(not-made p205)
(not-made p206)
(not-made p207)
(not-made p208)
(not-made p209)
(not-made p210)

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
(shipped o171)
(shipped o172)
(shipped o173)
(shipped o174)
(shipped o175)
(shipped o176)
(shipped o177)
(shipped o178)
(shipped o179)
(shipped o180)
(shipped o181)
(shipped o182)
(shipped o183)
(shipped o184)
(shipped o185)
(shipped o186)
(shipped o187)
(shipped o188)
(shipped o189)
(shipped o190)
(shipped o191)
(shipped o192)
(shipped o193)
(shipped o194)
(shipped o195)
(shipped o196)
(shipped o197)
(shipped o198)
(shipped o199)
(shipped o200)
(shipped o201)
(shipped o202)
(shipped o203)
(shipped o204)
(shipped o205)
(shipped o206)
(shipped o207)
(shipped o208)
(shipped o209)
(shipped o210)
))

(:metric minimize (total-cost))

)

