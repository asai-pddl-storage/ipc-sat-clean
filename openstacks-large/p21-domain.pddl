(define (domain openstacks-sequencedstrips-nonADL-nonNegated)
(:requirements :typing :action-costs)
(:types order product count)
(:constants 
 p1 p2 p3 p4 p5 p6 p7 p8 p9 p10 p11 p12 p13 p14 p15 p16 p17 p18 p19 p20 p21 p22 p23 p24 p25 p26 p27 p28 p29 p30 p31 p32 p33 p34 p35 p36 p37 p38 p39 p40 p41 p42 p43 p44 p45 p46 p47 p48 p49 p50 p51 p52 p53 p54 p55 p56 p57 p58 p59 p60 p61 p62 p63 p64 p65 p66 p67 p68 p69 p70 p71 p72 p73 p74 p75 p76 p77 p78 p79 p80 p81 p82 p83 p84 p85 p86 p87 p88 p89 p90 p91 p92 p93 p94 p95 p96 p97 p98 p99 p100 p101 p102 p103 p104 p105 p106 p107 p108 p109 p110 p111 p112 p113 p114 p115 p116 p117 p118 p119 p120 p121 p122 p123 p124 p125 p126 p127 p128 p129 p130 p131 p132 p133 p134 p135 p136 p137 p138 p139 p140 p141 p142 p143 p144 p145 p146 p147 p148 p149 p150 p151 p152 p153 p154 p155 p156 p157 p158 p159 p160 p161 p162 p163 p164 p165 p166 p167 p168 p169 p170 p171 p172 p173 p174 p175 p176 p177 p178 p179 p180 p181 p182 p183 p184 p185 p186 p187 p188 p189 p190 p191 p192 p193 p194 p195 p196 p197 p198 p199 p200 p201 p202 p203 p204 p205 p206 p207 p208 p209 p210 p211 p212 p213 p214 p215 p216 p217 p218 p219 p220 p221 p222 p223 p224 p225 p226 p227 p228 p229 p230 p231 p232 p233 p234 p235 p236 p237 p238 p239 p240 p241 p242 p243 p244 p245 p246 p247 p248 p249 p250 p251 p252 p253 p254 p255 p256 p257 p258 p259 p260 p261 p262 p263 p264 p265 p266 p267 p268 p269 p270 p271 p272 p273 p274 p275 p276 p277 p278 p279 p280 p281 p282 p283 p284 p285 p286 p287 p288 p289 p290 - product
 o1 o2 o3 o4 o5 o6 o7 o8 o9 o10 o11 o12 o13 o14 o15 o16 o17 o18 o19 o20 o21 o22 o23 o24 o25 o26 o27 o28 o29 o30 o31 o32 o33 o34 o35 o36 o37 o38 o39 o40 o41 o42 o43 o44 o45 o46 o47 o48 o49 o50 o51 o52 o53 o54 o55 o56 o57 o58 o59 o60 o61 o62 o63 o64 o65 o66 o67 o68 o69 o70 o71 o72 o73 o74 o75 o76 o77 o78 o79 o80 o81 o82 o83 o84 o85 o86 o87 o88 o89 o90 o91 o92 o93 o94 o95 o96 o97 o98 o99 o100 o101 o102 o103 o104 o105 o106 o107 o108 o109 o110 o111 o112 o113 o114 o115 o116 o117 o118 o119 o120 o121 o122 o123 o124 o125 o126 o127 o128 o129 o130 o131 o132 o133 o134 o135 o136 o137 o138 o139 o140 o141 o142 o143 o144 o145 o146 o147 o148 o149 o150 o151 o152 o153 o154 o155 o156 o157 o158 o159 o160 o161 o162 o163 o164 o165 o166 o167 o168 o169 o170 o171 o172 o173 o174 o175 o176 o177 o178 o179 o180 o181 o182 o183 o184 o185 o186 o187 o188 o189 o190 o191 o192 o193 o194 o195 o196 o197 o198 o199 o200 o201 o202 o203 o204 o205 o206 o207 o208 o209 o210 o211 o212 o213 o214 o215 o216 o217 o218 o219 o220 o221 o222 o223 o224 o225 o226 o227 o228 o229 o230 o231 o232 o233 o234 o235 o236 o237 o238 o239 o240 o241 o242 o243 o244 o245 o246 o247 o248 o249 o250 o251 o252 o253 o254 o255 o256 o257 o258 o259 o260 o261 o262 o263 o264 o265 o266 o267 o268 o269 o270 o271 o272 o273 o274 o275 o276 o277 o278 o279 o280 o281 o282 o283 o284 o285 o286 o287 o288 o289 o290 - order
)

(:predicates 
	(includes ?o - order ?p - product)
	(waiting ?o - order)
	(started ?o - order)
	(shipped ?o - order)
	(made ?p - product)
	(not-made ?p - product)
	(stacks-avail ?s - count)
	(next-count ?s ?ns - count)
)

(:functions
(total-cost)
)

(:action open-new-stack
:parameters (?open ?new-open - count)
:precondition (and (stacks-avail ?open)(next-count ?open ?new-open))
:effect (and (not (stacks-avail ?open))(stacks-avail ?new-open) (increase (total-cost) 1))
)

(:action start-order
:parameters (?o - order ?avail ?new-avail - count)
:precondition (and (waiting ?o)(stacks-avail ?avail)(next-count ?new-avail ?avail))
:effect (and (not (waiting ?o))(started ?o)(not (stacks-avail ?avail))(stacks-avail ?new-avail))
)

(:action make-product-p1
:parameters ()
:precondition (and (not-made p1)(started o24)(started o40)(started o47)(started o151)(started o211)(started o224)(started o274))
:effect (and (not (not-made p1)) (made p1))
)

(:action make-product-p2
:parameters ()
:precondition (and (not-made p2)(started o22)(started o50)(started o54)(started o65)(started o69)(started o78)(started o88)(started o129)(started o199)(started o269))
:effect (and (not (not-made p2)) (made p2))
)

(:action make-product-p3
:parameters ()
:precondition (and (not-made p3)(started o112)(started o127)(started o135)(started o165)(started o173)(started o216))
:effect (and (not (not-made p3)) (made p3))
)

(:action make-product-p4
:parameters ()
:precondition (and (not-made p4)(started o100)(started o158)(started o159)(started o193)(started o257)(started o265)(started o279))
:effect (and (not (not-made p4)) (made p4))
)

(:action make-product-p5
:parameters ()
:precondition (and (not-made p5)(started o56)(started o68)(started o108)(started o146)(started o197)(started o210)(started o226)(started o258))
:effect (and (not (not-made p5)) (made p5))
)

(:action make-product-p6
:parameters ()
:precondition (and (not-made p6)(started o16)(started o47)(started o110)(started o125)(started o173)(started o193)(started o257)(started o267))
:effect (and (not (not-made p6)) (made p6))
)

(:action make-product-p7
:parameters ()
:precondition (and (not-made p7)(started o51)(started o210)(started o216))
:effect (and (not (not-made p7)) (made p7))
)

(:action make-product-p8
:parameters ()
:precondition (and (not-made p8)(started o24)(started o83)(started o97)(started o140)(started o190)(started o192)(started o290))
:effect (and (not (not-made p8)) (made p8))
)

(:action make-product-p9
:parameters ()
:precondition (and (not-made p9)(started o17)(started o39)(started o42)(started o113)(started o136)(started o158)(started o189)(started o219)(started o235))
:effect (and (not (not-made p9)) (made p9))
)

(:action make-product-p10
:parameters ()
:precondition (and (not-made p10)(started o36)(started o37)(started o137)(started o164)(started o221)(started o234)(started o246)(started o276))
:effect (and (not (not-made p10)) (made p10))
)

(:action make-product-p11
:parameters ()
:precondition (and (not-made p11)(started o3)(started o101)(started o183)(started o217)(started o231)(started o233)(started o243))
:effect (and (not (not-made p11)) (made p11))
)

(:action make-product-p12
:parameters ()
:precondition (and (not-made p12)(started o121)(started o168))
:effect (and (not (not-made p12)) (made p12))
)

(:action make-product-p13
:parameters ()
:precondition (and (not-made p13)(started o10)(started o101)(started o117)(started o162)(started o172)(started o175)(started o224)(started o236)(started o282))
:effect (and (not (not-made p13)) (made p13))
)

(:action make-product-p14
:parameters ()
:precondition (and (not-made p14)(started o26)(started o92)(started o106)(started o130)(started o175))
:effect (and (not (not-made p14)) (made p14))
)

(:action make-product-p15
:parameters ()
:precondition (and (not-made p15)(started o6)(started o44)(started o260)(started o276))
:effect (and (not (not-made p15)) (made p15))
)

(:action make-product-p16
:parameters ()
:precondition (and (not-made p16)(started o76)(started o107)(started o168)(started o176)(started o178)(started o195))
:effect (and (not (not-made p16)) (made p16))
)

(:action make-product-p17
:parameters ()
:precondition (and (not-made p17)(started o10)(started o125)(started o143)(started o172)(started o188)(started o228)(started o249))
:effect (and (not (not-made p17)) (made p17))
)

(:action make-product-p18
:parameters ()
:precondition (and (not-made p18)(started o9)(started o12)(started o41)(started o99)(started o143)(started o188)(started o212)(started o279))
:effect (and (not (not-made p18)) (made p18))
)

(:action make-product-p19
:parameters ()
:precondition (and (not-made p19)(started o32)(started o101)(started o114)(started o121)(started o126)(started o181)(started o228)(started o280))
:effect (and (not (not-made p19)) (made p19))
)

(:action make-product-p20
:parameters ()
:precondition (and (not-made p20)(started o8)(started o56)(started o66)(started o168)(started o204)(started o208)(started o228)(started o243))
:effect (and (not (not-made p20)) (made p20))
)

(:action make-product-p21
:parameters ()
:precondition (and (not-made p21)(started o63)(started o67)(started o102)(started o153)(started o166)(started o175)(started o201)(started o206)(started o243))
:effect (and (not (not-made p21)) (made p21))
)

(:action make-product-p22
:parameters ()
:precondition (and (not-made p22)(started o62)(started o141)(started o195))
:effect (and (not (not-made p22)) (made p22))
)

(:action make-product-p23
:parameters ()
:precondition (and (not-made p23)(started o6)(started o164)(started o169)(started o183)(started o206)(started o215)(started o273))
:effect (and (not (not-made p23)) (made p23))
)

(:action make-product-p24
:parameters ()
:precondition (and (not-made p24)(started o52)(started o95)(started o107)(started o136)(started o152)(started o221)(started o274))
:effect (and (not (not-made p24)) (made p24))
)

(:action make-product-p25
:parameters ()
:precondition (and (not-made p25)(started o71)(started o104)(started o113)(started o129)(started o136)(started o164)(started o202)(started o236)(started o267)(started o278))
:effect (and (not (not-made p25)) (made p25))
)

(:action make-product-p26
:parameters ()
:precondition (and (not-made p26)(started o81)(started o192)(started o202)(started o230)(started o268)(started o270)(started o286))
:effect (and (not (not-made p26)) (made p26))
)

(:action make-product-p27
:parameters ()
:precondition (and (not-made p27)(started o41)(started o99)(started o225)(started o273))
:effect (and (not (not-made p27)) (made p27))
)

(:action make-product-p28
:parameters ()
:precondition (and (not-made p28)(started o33)(started o69)(started o96)(started o99)(started o235)(started o259)(started o260))
:effect (and (not (not-made p28)) (made p28))
)

(:action make-product-p29
:parameters ()
:precondition (and (not-made p29)(started o23)(started o59)(started o157)(started o212)(started o234))
:effect (and (not (not-made p29)) (made p29))
)

(:action make-product-p30
:parameters ()
:precondition (and (not-made p30)(started o20)(started o84)(started o124)(started o158)(started o217)(started o242)(started o261)(started o277))
:effect (and (not (not-made p30)) (made p30))
)

(:action make-product-p31
:parameters ()
:precondition (and (not-made p31)(started o69)(started o80)(started o157)(started o158)(started o180)(started o182)(started o222))
:effect (and (not (not-made p31)) (made p31))
)

(:action make-product-p32
:parameters ()
:precondition (and (not-made p32)(started o7)(started o78)(started o101)(started o143)(started o147)(started o245)(started o266))
:effect (and (not (not-made p32)) (made p32))
)

(:action make-product-p33
:parameters ()
:precondition (and (not-made p33)(started o32)(started o57)(started o72)(started o80)(started o224))
:effect (and (not (not-made p33)) (made p33))
)

(:action make-product-p34
:parameters ()
:precondition (and (not-made p34)(started o64)(started o81)(started o82)(started o111)(started o161)(started o204)(started o225))
:effect (and (not (not-made p34)) (made p34))
)

(:action make-product-p35
:parameters ()
:precondition (and (not-made p35)(started o49)(started o55)(started o73)(started o83)(started o170)(started o218)(started o241))
:effect (and (not (not-made p35)) (made p35))
)

(:action make-product-p36
:parameters ()
:precondition (and (not-made p36)(started o18)(started o109)(started o117)(started o158)(started o165)(started o169))
:effect (and (not (not-made p36)) (made p36))
)

(:action make-product-p37
:parameters ()
:precondition (and (not-made p37)(started o13)(started o56)(started o156)(started o160)(started o179)(started o189)(started o221)(started o290))
:effect (and (not (not-made p37)) (made p37))
)

(:action make-product-p38
:parameters ()
:precondition (and (not-made p38)(started o40)(started o49)(started o54)(started o132)(started o159)(started o275))
:effect (and (not (not-made p38)) (made p38))
)

(:action make-product-p39
:parameters ()
:precondition (and (not-made p39)(started o11)(started o26)(started o139)(started o144)(started o153)(started o193)(started o278))
:effect (and (not (not-made p39)) (made p39))
)

(:action make-product-p40
:parameters ()
:precondition (and (not-made p40)(started o10)(started o84)(started o88)(started o101)(started o115)(started o253))
:effect (and (not (not-made p40)) (made p40))
)

(:action make-product-p41
:parameters ()
:precondition (and (not-made p41)(started o4)(started o146)(started o188)(started o217))
:effect (and (not (not-made p41)) (made p41))
)

(:action make-product-p42
:parameters ()
:precondition (and (not-made p42)(started o11)(started o71)(started o77)(started o129)(started o133)(started o134)(started o219)(started o237)(started o252)(started o270))
:effect (and (not (not-made p42)) (made p42))
)

(:action make-product-p43
:parameters ()
:precondition (and (not-made p43)(started o25)(started o54)(started o61)(started o71)(started o74)(started o94)(started o156)(started o265)(started o273))
:effect (and (not (not-made p43)) (made p43))
)

(:action make-product-p44
:parameters ()
:precondition (and (not-made p44)(started o42)(started o71)(started o78)(started o117)(started o135)(started o260)(started o264))
:effect (and (not (not-made p44)) (made p44))
)

(:action make-product-p45
:parameters ()
:precondition (and (not-made p45)(started o40)(started o47)(started o79)(started o92)(started o108)(started o126)(started o137)(started o164)(started o201))
:effect (and (not (not-made p45)) (made p45))
)

(:action make-product-p46
:parameters ()
:precondition (and (not-made p46)(started o67)(started o101)(started o112)(started o239))
:effect (and (not (not-made p46)) (made p46))
)

(:action make-product-p47
:parameters ()
:precondition (and (not-made p47)(started o24)(started o61)(started o68)(started o90)(started o92)(started o98)(started o273))
:effect (and (not (not-made p47)) (made p47))
)

(:action make-product-p48
:parameters ()
:precondition (and (not-made p48)(started o25)(started o48)(started o229)(started o251))
:effect (and (not (not-made p48)) (made p48))
)

(:action make-product-p49
:parameters ()
:precondition (and (not-made p49)(started o23)(started o119)(started o135)(started o178)(started o187)(started o216)(started o251))
:effect (and (not (not-made p49)) (made p49))
)

(:action make-product-p50
:parameters ()
:precondition (and (not-made p50)(started o3)(started o54)(started o196)(started o197)(started o207)(started o221)(started o243)(started o270))
:effect (and (not (not-made p50)) (made p50))
)

(:action make-product-p51
:parameters ()
:precondition (and (not-made p51)(started o57)(started o152)(started o225)(started o242))
:effect (and (not (not-made p51)) (made p51))
)

(:action make-product-p52
:parameters ()
:precondition (and (not-made p52)(started o1)(started o61)(started o65)(started o67)(started o75))
:effect (and (not (not-made p52)) (made p52))
)

(:action make-product-p53
:parameters ()
:precondition (and (not-made p53)(started o18)(started o36)(started o141)(started o185)(started o194)(started o234)(started o247)(started o250))
:effect (and (not (not-made p53)) (made p53))
)

(:action make-product-p54
:parameters ()
:precondition (and (not-made p54)(started o7)(started o32)(started o74)(started o94)(started o103)(started o130)(started o196)(started o213)(started o221))
:effect (and (not (not-made p54)) (made p54))
)

(:action make-product-p55
:parameters ()
:precondition (and (not-made p55)(started o27)(started o31)(started o73)(started o87)(started o162)(started o187))
:effect (and (not (not-made p55)) (made p55))
)

(:action make-product-p56
:parameters ()
:precondition (and (not-made p56)(started o78)(started o97)(started o181)(started o244))
:effect (and (not (not-made p56)) (made p56))
)

(:action make-product-p57
:parameters ()
:precondition (and (not-made p57)(started o26)(started o52)(started o101)(started o191)(started o223))
:effect (and (not (not-made p57)) (made p57))
)

(:action make-product-p58
:parameters ()
:precondition (and (not-made p58)(started o14)(started o54)(started o111)(started o128)(started o162)(started o193)(started o224))
:effect (and (not (not-made p58)) (made p58))
)

(:action make-product-p59
:parameters ()
:precondition (and (not-made p59)(started o66)(started o107)(started o176)(started o182)(started o289))
:effect (and (not (not-made p59)) (made p59))
)

(:action make-product-p60
:parameters ()
:precondition (and (not-made p60)(started o36)(started o66)(started o77)(started o81)(started o91)(started o180))
:effect (and (not (not-made p60)) (made p60))
)

(:action make-product-p61
:parameters ()
:precondition (and (not-made p61)(started o7)(started o8)(started o174)(started o188)(started o189))
:effect (and (not (not-made p61)) (made p61))
)

(:action make-product-p62
:parameters ()
:precondition (and (not-made p62)(started o68)(started o205)(started o242)(started o245))
:effect (and (not (not-made p62)) (made p62))
)

(:action make-product-p63
:parameters ()
:precondition (and (not-made p63)(started o16)(started o22)(started o31)(started o53)(started o60)(started o100)(started o102)(started o149)(started o162)(started o191)(started o218))
:effect (and (not (not-made p63)) (made p63))
)

(:action make-product-p64
:parameters ()
:precondition (and (not-made p64)(started o4)(started o14)(started o167)(started o192)(started o209)(started o259)(started o261))
:effect (and (not (not-made p64)) (made p64))
)

(:action make-product-p65
:parameters ()
:precondition (and (not-made p65)(started o32)(started o66)(started o121)(started o128)(started o149))
:effect (and (not (not-made p65)) (made p65))
)

(:action make-product-p66
:parameters ()
:precondition (and (not-made p66)(started o115)(started o151)(started o157)(started o211)(started o263)(started o286))
:effect (and (not (not-made p66)) (made p66))
)

(:action make-product-p67
:parameters ()
:precondition (and (not-made p67)(started o22)(started o114)(started o180)(started o205)(started o215))
:effect (and (not (not-made p67)) (made p67))
)

(:action make-product-p68
:parameters ()
:precondition (and (not-made p68)(started o5)(started o44)(started o56)(started o178)(started o202)(started o252)(started o287))
:effect (and (not (not-made p68)) (made p68))
)

(:action make-product-p69
:parameters ()
:precondition (and (not-made p69)(started o5)(started o115)(started o128)(started o142)(started o145)(started o158)(started o180)(started o214)(started o226)(started o237))
:effect (and (not (not-made p69)) (made p69))
)

(:action make-product-p70
:parameters ()
:precondition (and (not-made p70)(started o9)(started o15)(started o33)(started o45)(started o102)(started o103)(started o167)(started o198)(started o273)(started o287)(started o288))
:effect (and (not (not-made p70)) (made p70))
)

(:action make-product-p71
:parameters ()
:precondition (and (not-made p71)(started o20)(started o25)(started o30)(started o181))
:effect (and (not (not-made p71)) (made p71))
)

(:action make-product-p72
:parameters ()
:precondition (and (not-made p72)(started o5)(started o36)(started o81)(started o131)(started o245)(started o264))
:effect (and (not (not-made p72)) (made p72))
)

(:action make-product-p73
:parameters ()
:precondition (and (not-made p73)(started o1)(started o22)(started o26)(started o28)(started o30)(started o47)(started o54)(started o159)(started o164))
:effect (and (not (not-made p73)) (made p73))
)

(:action make-product-p74
:parameters ()
:precondition (and (not-made p74)(started o35)(started o60)(started o76)(started o95)(started o140)(started o145)(started o176)(started o229))
:effect (and (not (not-made p74)) (made p74))
)

(:action make-product-p75
:parameters ()
:precondition (and (not-made p75)(started o85)(started o122)(started o127)(started o162)(started o212)(started o216)(started o290))
:effect (and (not (not-made p75)) (made p75))
)

(:action make-product-p76
:parameters ()
:precondition (and (not-made p76)(started o34)(started o83)(started o143)(started o158)(started o194)(started o217)(started o261))
:effect (and (not (not-made p76)) (made p76))
)

(:action make-product-p77
:parameters ()
:precondition (and (not-made p77)(started o25)(started o118)(started o231)(started o254))
:effect (and (not (not-made p77)) (made p77))
)

(:action make-product-p78
:parameters ()
:precondition (and (not-made p78)(started o38)(started o51)(started o165)(started o275))
:effect (and (not (not-made p78)) (made p78))
)

(:action make-product-p79
:parameters ()
:precondition (and (not-made p79)(started o12)(started o33)(started o68)(started o71)(started o72)(started o90)(started o146)(started o200)(started o215)(started o256)(started o285))
:effect (and (not (not-made p79)) (made p79))
)

(:action make-product-p80
:parameters ()
:precondition (and (not-made p80)(started o40)(started o67)(started o94)(started o114)(started o116)(started o126)(started o166)(started o182)(started o243)(started o246))
:effect (and (not (not-made p80)) (made p80))
)

(:action make-product-p81
:parameters ()
:precondition (and (not-made p81)(started o6)(started o69)(started o127)(started o146)(started o207)(started o216)(started o258)(started o276))
:effect (and (not (not-made p81)) (made p81))
)

(:action make-product-p82
:parameters ()
:precondition (and (not-made p82)(started o38)(started o60)(started o93)(started o113)(started o145)(started o161)(started o179)(started o182)(started o196)(started o207)(started o215))
:effect (and (not (not-made p82)) (made p82))
)

(:action make-product-p83
:parameters ()
:precondition (and (not-made p83)(started o60)(started o169)(started o193)(started o271)(started o290))
:effect (and (not (not-made p83)) (made p83))
)

(:action make-product-p84
:parameters ()
:precondition (and (not-made p84)(started o1)(started o90)(started o173)(started o209)(started o234)(started o235)(started o264)(started o283))
:effect (and (not (not-made p84)) (made p84))
)

(:action make-product-p85
:parameters ()
:precondition (and (not-made p85)(started o36)(started o51)(started o54)(started o82)(started o145)(started o167)(started o169)(started o200)(started o219)(started o242)(started o254)(started o277))
:effect (and (not (not-made p85)) (made p85))
)

(:action make-product-p86
:parameters ()
:precondition (and (not-made p86)(started o9)(started o41)(started o74)(started o215)(started o264))
:effect (and (not (not-made p86)) (made p86))
)

(:action make-product-p87
:parameters ()
:precondition (and (not-made p87)(started o72)(started o100)(started o103)(started o169)(started o239)(started o268))
:effect (and (not (not-made p87)) (made p87))
)

(:action make-product-p88
:parameters ()
:precondition (and (not-made p88)(started o21)(started o28)(started o66)(started o70)(started o150)(started o165)(started o265))
:effect (and (not (not-made p88)) (made p88))
)

(:action make-product-p89
:parameters ()
:precondition (and (not-made p89)(started o88)(started o105)(started o140)(started o198)(started o226)(started o227)(started o251))
:effect (and (not (not-made p89)) (made p89))
)

(:action make-product-p90
:parameters ()
:precondition (and (not-made p90)(started o124)(started o138)(started o151)(started o167)(started o187)(started o249)(started o252))
:effect (and (not (not-made p90)) (made p90))
)

(:action make-product-p91
:parameters ()
:precondition (and (not-made p91)(started o45)(started o56)(started o183)(started o271)(started o273)(started o290))
:effect (and (not (not-made p91)) (made p91))
)

(:action make-product-p92
:parameters ()
:precondition (and (not-made p92)(started o8)(started o110)(started o123)(started o223)(started o236)(started o256))
:effect (and (not (not-made p92)) (made p92))
)

(:action make-product-p93
:parameters ()
:precondition (and (not-made p93)(started o10)(started o87)(started o147)(started o164)(started o253)(started o264))
:effect (and (not (not-made p93)) (made p93))
)

(:action make-product-p94
:parameters ()
:precondition (and (not-made p94)(started o1)(started o70)(started o81)(started o109)(started o239))
:effect (and (not (not-made p94)) (made p94))
)

(:action make-product-p95
:parameters ()
:precondition (and (not-made p95)(started o21)(started o49)(started o99)(started o194)(started o228))
:effect (and (not (not-made p95)) (made p95))
)

(:action make-product-p96
:parameters ()
:precondition (and (not-made p96)(started o60)(started o79)(started o86)(started o105)(started o114)(started o207)(started o248)(started o287))
:effect (and (not (not-made p96)) (made p96))
)

(:action make-product-p97
:parameters ()
:precondition (and (not-made p97)(started o20)(started o103)(started o104)(started o106)(started o176)(started o206)(started o219)(started o258)(started o288))
:effect (and (not (not-made p97)) (made p97))
)

(:action make-product-p98
:parameters ()
:precondition (and (not-made p98)(started o9)(started o51)(started o136)(started o174)(started o222)(started o290))
:effect (and (not (not-made p98)) (made p98))
)

(:action make-product-p99
:parameters ()
:precondition (and (not-made p99)(started o43)(started o89)(started o190)(started o192))
:effect (and (not (not-made p99)) (made p99))
)

(:action make-product-p100
:parameters ()
:precondition (and (not-made p100)(started o22)(started o77)(started o107)(started o134)(started o191)(started o257)(started o272))
:effect (and (not (not-made p100)) (made p100))
)

(:action make-product-p101
:parameters ()
:precondition (and (not-made p101)(started o1)(started o9)(started o32)(started o41)(started o45)(started o53)(started o175)(started o223)(started o251)(started o275)(started o287))
:effect (and (not (not-made p101)) (made p101))
)

(:action make-product-p102
:parameters ()
:precondition (and (not-made p102)(started o14)(started o29)(started o43)(started o57)(started o96))
:effect (and (not (not-made p102)) (made p102))
)

(:action make-product-p103
:parameters ()
:precondition (and (not-made p103)(started o2)(started o33)(started o36)(started o98)(started o108)(started o115)(started o146)(started o208))
:effect (and (not (not-made p103)) (made p103))
)

(:action make-product-p104
:parameters ()
:precondition (and (not-made p104)(started o3)(started o16)(started o35)(started o36)(started o39)(started o71)(started o134)(started o164)(started o222)(started o270))
:effect (and (not (not-made p104)) (made p104))
)

(:action make-product-p105
:parameters ()
:precondition (and (not-made p105)(started o4)(started o6)(started o84)(started o97)(started o108)(started o168)(started o205)(started o263)(started o286))
:effect (and (not (not-made p105)) (made p105))
)

(:action make-product-p106
:parameters ()
:precondition (and (not-made p106)(started o81)(started o212)(started o245))
:effect (and (not (not-made p106)) (made p106))
)

(:action make-product-p107
:parameters ()
:precondition (and (not-made p107)(started o25)(started o51)(started o102)(started o185)(started o188)(started o225)(started o242)(started o289))
:effect (and (not (not-made p107)) (made p107))
)

(:action make-product-p108
:parameters ()
:precondition (and (not-made p108)(started o14)(started o53)(started o102)(started o118)(started o144)(started o160)(started o163)(started o169)(started o175)(started o177))
:effect (and (not (not-made p108)) (made p108))
)

(:action make-product-p109
:parameters ()
:precondition (and (not-made p109)(started o39)(started o102)(started o103)(started o109)(started o148)(started o194)(started o239))
:effect (and (not (not-made p109)) (made p109))
)

(:action make-product-p110
:parameters ()
:precondition (and (not-made p110)(started o86)(started o129)(started o136)(started o158)(started o170)(started o265)(started o269))
:effect (and (not (not-made p110)) (made p110))
)

(:action make-product-p111
:parameters ()
:precondition (and (not-made p111)(started o2)(started o5)(started o22)(started o42)(started o195)(started o202)(started o212)(started o237)(started o287)(started o289)(started o290))
:effect (and (not (not-made p111)) (made p111))
)

(:action make-product-p112
:parameters ()
:precondition (and (not-made p112)(started o18)(started o48)(started o73)(started o107)(started o114)(started o117)(started o177)(started o194))
:effect (and (not (not-made p112)) (made p112))
)

(:action make-product-p113
:parameters ()
:precondition (and (not-made p113)(started o55)(started o73)(started o130)(started o142)(started o167)(started o203)(started o209)(started o287))
:effect (and (not (not-made p113)) (made p113))
)

(:action make-product-p114
:parameters ()
:precondition (and (not-made p114)(started o46)(started o157)(started o167)(started o251)(started o276)(started o287))
:effect (and (not (not-made p114)) (made p114))
)

(:action make-product-p115
:parameters ()
:precondition (and (not-made p115)(started o18)(started o51)(started o53)(started o64)(started o73)(started o74)(started o252)(started o255))
:effect (and (not (not-made p115)) (made p115))
)

(:action make-product-p116
:parameters ()
:precondition (and (not-made p116)(started o54)(started o145)(started o189)(started o263))
:effect (and (not (not-made p116)) (made p116))
)

(:action make-product-p117
:parameters ()
:precondition (and (not-made p117)(started o246))
:effect (and (not (not-made p117)) (made p117))
)

(:action make-product-p118
:parameters ()
:precondition (and (not-made p118)(started o139)(started o164)(started o176)(started o182)(started o275)(started o282)(started o289))
:effect (and (not (not-made p118)) (made p118))
)

(:action make-product-p119
:parameters ()
:precondition (and (not-made p119)(started o27)(started o31)(started o142)(started o156)(started o157)(started o250)(started o258)(started o269)(started o271))
:effect (and (not (not-made p119)) (made p119))
)

(:action make-product-p120
:parameters ()
:precondition (and (not-made p120)(started o14)(started o63)(started o110)(started o151)(started o249)(started o263))
:effect (and (not (not-made p120)) (made p120))
)

(:action make-product-p121
:parameters ()
:precondition (and (not-made p121)(started o238)(started o247))
:effect (and (not (not-made p121)) (made p121))
)

(:action make-product-p122
:parameters ()
:precondition (and (not-made p122)(started o26)(started o102)(started o129)(started o140)(started o164)(started o195))
:effect (and (not (not-made p122)) (made p122))
)

(:action make-product-p123
:parameters ()
:precondition (and (not-made p123)(started o23)(started o83)(started o90)(started o123)(started o135)(started o159)(started o192)(started o246))
:effect (and (not (not-made p123)) (made p123))
)

(:action make-product-p124
:parameters ()
:precondition (and (not-made p124)(started o133)(started o156))
:effect (and (not (not-made p124)) (made p124))
)

(:action make-product-p125
:parameters ()
:precondition (and (not-made p125)(started o144)(started o183)(started o266)(started o278)(started o281))
:effect (and (not (not-made p125)) (made p125))
)

(:action make-product-p126
:parameters ()
:precondition (and (not-made p126)(started o12)(started o39)(started o60)(started o73)(started o96))
:effect (and (not (not-made p126)) (made p126))
)

(:action make-product-p127
:parameters ()
:precondition (and (not-made p127)(started o60)(started o81)(started o107)(started o147)(started o153)(started o157)(started o183)(started o238)(started o275))
:effect (and (not (not-made p127)) (made p127))
)

(:action make-product-p128
:parameters ()
:precondition (and (not-made p128)(started o29)(started o63)(started o118)(started o162)(started o221)(started o259))
:effect (and (not (not-made p128)) (made p128))
)

(:action make-product-p129
:parameters ()
:precondition (and (not-made p129)(started o18)(started o26)(started o45)(started o167)(started o196)(started o203)(started o228)(started o283))
:effect (and (not (not-made p129)) (made p129))
)

(:action make-product-p130
:parameters ()
:precondition (and (not-made p130)(started o8)(started o14)(started o67)(started o105)(started o116)(started o121)(started o127)(started o160)(started o223)(started o242)(started o253))
:effect (and (not (not-made p130)) (made p130))
)

(:action make-product-p131
:parameters ()
:precondition (and (not-made p131)(started o100)(started o172)(started o181)(started o230)(started o245)(started o248)(started o257))
:effect (and (not (not-made p131)) (made p131))
)

(:action make-product-p132
:parameters ()
:precondition (and (not-made p132)(started o85)(started o104)(started o131)(started o136)(started o140)(started o145)(started o147)(started o187)(started o192)(started o209)(started o242)(started o245))
:effect (and (not (not-made p132)) (made p132))
)

(:action make-product-p133
:parameters ()
:precondition (and (not-made p133)(started o42)(started o53)(started o65)(started o73)(started o108)(started o194)(started o217))
:effect (and (not (not-made p133)) (made p133))
)

(:action make-product-p134
:parameters ()
:precondition (and (not-made p134)(started o17)(started o57)(started o92)(started o155)(started o196)(started o213)(started o224))
:effect (and (not (not-made p134)) (made p134))
)

(:action make-product-p135
:parameters ()
:precondition (and (not-made p135)(started o42)(started o56)(started o101)(started o158)(started o174)(started o181)(started o183)(started o190)(started o235)(started o250)(started o264)(started o279))
:effect (and (not (not-made p135)) (made p135))
)

(:action make-product-p136
:parameters ()
:precondition (and (not-made p136)(started o3)(started o51)(started o79)(started o81)(started o93)(started o94)(started o108)(started o186)(started o214)(started o243))
:effect (and (not (not-made p136)) (made p136))
)

(:action make-product-p137
:parameters ()
:precondition (and (not-made p137)(started o111)(started o162)(started o207))
:effect (and (not (not-made p137)) (made p137))
)

(:action make-product-p138
:parameters ()
:precondition (and (not-made p138)(started o30)(started o83)(started o134)(started o176)(started o188)(started o189)(started o217)(started o222)(started o238)(started o258))
:effect (and (not (not-made p138)) (made p138))
)

(:action make-product-p139
:parameters ()
:precondition (and (not-made p139)(started o25)(started o63)(started o190)(started o203))
:effect (and (not (not-made p139)) (made p139))
)

(:action make-product-p140
:parameters ()
:precondition (and (not-made p140)(started o15)(started o27)(started o103)(started o106)(started o134)(started o199)(started o211)(started o252)(started o258)(started o282))
:effect (and (not (not-made p140)) (made p140))
)

(:action make-product-p141
:parameters ()
:precondition (and (not-made p141)(started o13)(started o27)(started o111)(started o116)(started o185)(started o236))
:effect (and (not (not-made p141)) (made p141))
)

(:action make-product-p142
:parameters ()
:precondition (and (not-made p142)(started o1)(started o16)(started o52)(started o224)(started o228)(started o272))
:effect (and (not (not-made p142)) (made p142))
)

(:action make-product-p143
:parameters ()
:precondition (and (not-made p143)(started o4)(started o72)(started o104)(started o176)(started o205)(started o211)(started o247)(started o286))
:effect (and (not (not-made p143)) (made p143))
)

(:action make-product-p144
:parameters ()
:precondition (and (not-made p144)(started o12)(started o117)(started o127)(started o148)(started o149)(started o154)(started o173)(started o213)(started o218))
:effect (and (not (not-made p144)) (made p144))
)

(:action make-product-p145
:parameters ()
:precondition (and (not-made p145)(started o36)(started o174)(started o177)(started o199))
:effect (and (not (not-made p145)) (made p145))
)

(:action make-product-p146
:parameters ()
:precondition (and (not-made p146)(started o9)(started o63)(started o78)(started o114)(started o139)(started o140)(started o174)(started o264))
:effect (and (not (not-made p146)) (made p146))
)

(:action make-product-p147
:parameters ()
:precondition (and (not-made p147)(started o83)(started o90)(started o169)(started o192)(started o194)(started o252)(started o259))
:effect (and (not (not-made p147)) (made p147))
)

(:action make-product-p148
:parameters ()
:precondition (and (not-made p148)(started o30)(started o64)(started o98)(started o189)(started o281))
:effect (and (not (not-made p148)) (made p148))
)

(:action make-product-p149
:parameters ()
:precondition (and (not-made p149)(started o42)(started o80)(started o202)(started o210)(started o224)(started o282))
:effect (and (not (not-made p149)) (made p149))
)

(:action make-product-p150
:parameters ()
:precondition (and (not-made p150)(started o19)(started o51)(started o72)(started o144)(started o186)(started o224)(started o251)(started o257))
:effect (and (not (not-made p150)) (made p150))
)

(:action make-product-p151
:parameters ()
:precondition (and (not-made p151)(started o21)(started o48)(started o78)(started o105)(started o115)(started o165)(started o196))
:effect (and (not (not-made p151)) (made p151))
)

(:action make-product-p152
:parameters ()
:precondition (and (not-made p152)(started o5)(started o38)(started o41)(started o46)(started o61)(started o139)(started o169)(started o185)(started o214)(started o233))
:effect (and (not (not-made p152)) (made p152))
)

(:action make-product-p153
:parameters ()
:precondition (and (not-made p153)(started o23)(started o89)(started o114)(started o182)(started o198)(started o279))
:effect (and (not (not-made p153)) (made p153))
)

(:action make-product-p154
:parameters ()
:precondition (and (not-made p154)(started o10)(started o68)(started o178)(started o190))
:effect (and (not (not-made p154)) (made p154))
)

(:action make-product-p155
:parameters ()
:precondition (and (not-made p155)(started o6)(started o9)(started o53)(started o58)(started o95)(started o132)(started o275))
:effect (and (not (not-made p155)) (made p155))
)

(:action make-product-p156
:parameters ()
:precondition (and (not-made p156)(started o17)(started o33)(started o43)(started o72)(started o136)(started o183)(started o207)(started o240)(started o255)(started o263))
:effect (and (not (not-made p156)) (made p156))
)

(:action make-product-p157
:parameters ()
:precondition (and (not-made p157)(started o78)(started o79)(started o116)(started o121)(started o134)(started o170)(started o182)(started o264))
:effect (and (not (not-made p157)) (made p157))
)

(:action make-product-p158
:parameters ()
:precondition (and (not-made p158)(started o18)(started o250)(started o289))
:effect (and (not (not-made p158)) (made p158))
)

(:action make-product-p159
:parameters ()
:precondition (and (not-made p159)(started o40)(started o80)(started o88)(started o124)(started o130)(started o139)(started o154)(started o166)(started o183)(started o247)(started o261))
:effect (and (not (not-made p159)) (made p159))
)

(:action make-product-p160
:parameters ()
:precondition (and (not-made p160)(started o1)(started o123)(started o129)(started o140)(started o144)(started o155)(started o280))
:effect (and (not (not-made p160)) (made p160))
)

(:action make-product-p161
:parameters ()
:precondition (and (not-made p161)(started o88)(started o190))
:effect (and (not (not-made p161)) (made p161))
)

(:action make-product-p162
:parameters ()
:precondition (and (not-made p162)(started o18)(started o19)(started o36)(started o77)(started o91)(started o170)(started o236))
:effect (and (not (not-made p162)) (made p162))
)

(:action make-product-p163
:parameters ()
:precondition (and (not-made p163)(started o46)(started o194)(started o246)(started o269))
:effect (and (not (not-made p163)) (made p163))
)

(:action make-product-p164
:parameters ()
:precondition (and (not-made p164)(started o58)(started o98)(started o155))
:effect (and (not (not-made p164)) (made p164))
)

(:action make-product-p165
:parameters ()
:precondition (and (not-made p165)(started o10)(started o40)(started o58)(started o65)(started o67)(started o89)(started o142)(started o150)(started o165)(started o180)(started o246)(started o248)(started o257))
:effect (and (not (not-made p165)) (made p165))
)

(:action make-product-p166
:parameters ()
:precondition (and (not-made p166)(started o126)(started o223)(started o235)(started o264))
:effect (and (not (not-made p166)) (made p166))
)

(:action make-product-p167
:parameters ()
:precondition (and (not-made p167)(started o34)(started o91)(started o94)(started o127)(started o184))
:effect (and (not (not-made p167)) (made p167))
)

(:action make-product-p168
:parameters ()
:precondition (and (not-made p168)(started o29)(started o106)(started o107)(started o144))
:effect (and (not (not-made p168)) (made p168))
)

(:action make-product-p169
:parameters ()
:precondition (and (not-made p169)(started o2)(started o18)(started o113)(started o142)(started o206)(started o220)(started o233)(started o240)(started o249))
:effect (and (not (not-made p169)) (made p169))
)

(:action make-product-p170
:parameters ()
:precondition (and (not-made p170)(started o9)(started o32)(started o99)(started o224)(started o236))
:effect (and (not (not-made p170)) (made p170))
)

(:action make-product-p171
:parameters ()
:precondition (and (not-made p171)(started o21)(started o62)(started o67)(started o123)(started o162))
:effect (and (not (not-made p171)) (made p171))
)

(:action make-product-p172
:parameters ()
:precondition (and (not-made p172)(started o48)(started o203)(started o207)(started o243))
:effect (and (not (not-made p172)) (made p172))
)

(:action make-product-p173
:parameters ()
:precondition (and (not-made p173)(started o13)(started o33)(started o49)(started o113)(started o114)(started o134)(started o188)(started o198)(started o246))
:effect (and (not (not-made p173)) (made p173))
)

(:action make-product-p174
:parameters ()
:precondition (and (not-made p174)(started o3)(started o106)(started o107)(started o119)(started o166)(started o177)(started o208)(started o266)(started o275)(started o285))
:effect (and (not (not-made p174)) (made p174))
)

(:action make-product-p175
:parameters ()
:precondition (and (not-made p175)(started o3)(started o27)(started o48)(started o83)(started o136)(started o198)(started o243)(started o261))
:effect (and (not (not-made p175)) (made p175))
)

(:action make-product-p176
:parameters ()
:precondition (and (not-made p176)(started o24)(started o31)(started o87)(started o106)(started o134)(started o182))
:effect (and (not (not-made p176)) (made p176))
)

(:action make-product-p177
:parameters ()
:precondition (and (not-made p177)(started o10)(started o21)(started o65)(started o95)(started o223))
:effect (and (not (not-made p177)) (made p177))
)

(:action make-product-p178
:parameters ()
:precondition (and (not-made p178)(started o36)(started o71)(started o76)(started o205)(started o243)(started o257)(started o276)(started o282))
:effect (and (not (not-made p178)) (made p178))
)

(:action make-product-p179
:parameters ()
:precondition (and (not-made p179)(started o18)(started o22)(started o38)(started o98)(started o176)(started o220)(started o256))
:effect (and (not (not-made p179)) (made p179))
)

(:action make-product-p180
:parameters ()
:precondition (and (not-made p180)(started o93)(started o99)(started o149)(started o161)(started o185)(started o243)(started o270)(started o285))
:effect (and (not (not-made p180)) (made p180))
)

(:action make-product-p181
:parameters ()
:precondition (and (not-made p181)(started o40)(started o54)(started o72)(started o175)(started o193)(started o250))
:effect (and (not (not-made p181)) (made p181))
)

(:action make-product-p182
:parameters ()
:precondition (and (not-made p182)(started o46)(started o76)(started o129))
:effect (and (not (not-made p182)) (made p182))
)

(:action make-product-p183
:parameters ()
:precondition (and (not-made p183)(started o33)(started o34)(started o122)(started o245))
:effect (and (not (not-made p183)) (made p183))
)

(:action make-product-p184
:parameters ()
:precondition (and (not-made p184)(started o10)(started o91)(started o231)(started o241)(started o280))
:effect (and (not (not-made p184)) (made p184))
)

(:action make-product-p185
:parameters ()
:precondition (and (not-made p185)(started o1)(started o95)(started o108)(started o147)(started o197))
:effect (and (not (not-made p185)) (made p185))
)

(:action make-product-p186
:parameters ()
:precondition (and (not-made p186)(started o11)(started o46)(started o65)(started o142)(started o146)(started o219)(started o221))
:effect (and (not (not-made p186)) (made p186))
)

(:action make-product-p187
:parameters ()
:precondition (and (not-made p187)(started o29)(started o188))
:effect (and (not (not-made p187)) (made p187))
)

(:action make-product-p188
:parameters ()
:precondition (and (not-made p188)(started o50)(started o88)(started o184)(started o185)(started o193)(started o196)(started o245)(started o262))
:effect (and (not (not-made p188)) (made p188))
)

(:action make-product-p189
:parameters ()
:precondition (and (not-made p189)(started o26)(started o41)(started o50)(started o61)(started o77)(started o131)(started o135)(started o177)(started o224)(started o267))
:effect (and (not (not-made p189)) (made p189))
)

(:action make-product-p190
:parameters ()
:precondition (and (not-made p190)(started o22)(started o26)(started o41)(started o48)(started o71)(started o74)(started o133)(started o139)(started o214)(started o238)(started o254)(started o273))
:effect (and (not (not-made p190)) (made p190))
)

(:action make-product-p191
:parameters ()
:precondition (and (not-made p191)(started o2)(started o5)(started o26)(started o80)(started o124)(started o161)(started o233)(started o275))
:effect (and (not (not-made p191)) (made p191))
)

(:action make-product-p192
:parameters ()
:precondition (and (not-made p192)(started o63)(started o74)(started o132)(started o210))
:effect (and (not (not-made p192)) (made p192))
)

(:action make-product-p193
:parameters ()
:precondition (and (not-made p193)(started o11)(started o22)(started o47)(started o77)(started o95)(started o205)(started o246)(started o251)(started o262))
:effect (and (not (not-made p193)) (made p193))
)

(:action make-product-p194
:parameters ()
:precondition (and (not-made p194)(started o37)(started o49)(started o71)(started o134)(started o188)(started o191)(started o197)(started o232)(started o251))
:effect (and (not (not-made p194)) (made p194))
)

(:action make-product-p195
:parameters ()
:precondition (and (not-made p195)(started o25)(started o44)(started o69)(started o91)(started o96)(started o168)(started o201)(started o222)(started o224))
:effect (and (not (not-made p195)) (made p195))
)

(:action make-product-p196
:parameters ()
:precondition (and (not-made p196)(started o13)(started o45)(started o73)(started o233)(started o279))
:effect (and (not (not-made p196)) (made p196))
)

(:action make-product-p197
:parameters ()
:precondition (and (not-made p197)(started o45)(started o77)(started o93)(started o97)(started o111)(started o132)(started o170)(started o221)(started o250))
:effect (and (not (not-made p197)) (made p197))
)

(:action make-product-p198
:parameters ()
:precondition (and (not-made p198)(started o8)(started o33)(started o49)(started o58)(started o74)(started o147)(started o158)(started o160)(started o164)(started o233)(started o254)(started o290))
:effect (and (not (not-made p198)) (made p198))
)

(:action make-product-p199
:parameters ()
:precondition (and (not-made p199)(started o31)(started o49)(started o82)(started o104)(started o178)(started o233))
:effect (and (not (not-made p199)) (made p199))
)

(:action make-product-p200
:parameters ()
:precondition (and (not-made p200)(started o31)(started o40)(started o62)(started o125)(started o137)(started o141)(started o173)(started o191))
:effect (and (not (not-made p200)) (made p200))
)

(:action make-product-p201
:parameters ()
:precondition (and (not-made p201)(started o106)(started o108)(started o247))
:effect (and (not (not-made p201)) (made p201))
)

(:action make-product-p202
:parameters ()
:precondition (and (not-made p202)(started o21)(started o22)(started o30)(started o93)(started o131)(started o134)(started o152)(started o160)(started o201)(started o218)(started o228))
:effect (and (not (not-made p202)) (made p202))
)

(:action make-product-p203
:parameters ()
:precondition (and (not-made p203)(started o35)(started o36)(started o278)(started o289))
:effect (and (not (not-made p203)) (made p203))
)

(:action make-product-p204
:parameters ()
:precondition (and (not-made p204)(started o56)(started o60)(started o128)(started o157)(started o170)(started o190)(started o206)(started o222)(started o288))
:effect (and (not (not-made p204)) (made p204))
)

(:action make-product-p205
:parameters ()
:precondition (and (not-made p205)(started o11)(started o17)(started o137)(started o141)(started o193)(started o227)(started o239)(started o270))
:effect (and (not (not-made p205)) (made p205))
)

(:action make-product-p206
:parameters ()
:precondition (and (not-made p206)(started o36)(started o48)(started o91)(started o95)(started o146)(started o219)(started o220)(started o289))
:effect (and (not (not-made p206)) (made p206))
)

(:action make-product-p207
:parameters ()
:precondition (and (not-made p207)(started o101)(started o111)(started o133)(started o144)(started o172)(started o255))
:effect (and (not (not-made p207)) (made p207))
)

(:action make-product-p208
:parameters ()
:precondition (and (not-made p208)(started o42)(started o96)(started o145)(started o151))
:effect (and (not (not-made p208)) (made p208))
)

(:action make-product-p209
:parameters ()
:precondition (and (not-made p209)(started o75)(started o139)(started o287)(started o288))
:effect (and (not (not-made p209)) (made p209))
)

(:action make-product-p210
:parameters ()
:precondition (and (not-made p210)(started o5)(started o121)(started o145)(started o164)(started o185)(started o191)(started o218)(started o270))
:effect (and (not (not-made p210)) (made p210))
)

(:action make-product-p211
:parameters ()
:precondition (and (not-made p211)(started o24)(started o63)(started o100)(started o225)(started o236)(started o268))
:effect (and (not (not-made p211)) (made p211))
)

(:action make-product-p212
:parameters ()
:precondition (and (not-made p212)(started o6)(started o66)(started o191)(started o194)(started o262)(started o277))
:effect (and (not (not-made p212)) (made p212))
)

(:action make-product-p213
:parameters ()
:precondition (and (not-made p213)(started o12)(started o77)(started o107)(started o127)(started o162)(started o179)(started o229)(started o234)(started o280)(started o287))
:effect (and (not (not-made p213)) (made p213))
)

(:action make-product-p214
:parameters ()
:precondition (and (not-made p214)(started o3)(started o54)(started o149)(started o164)(started o215))
:effect (and (not (not-made p214)) (made p214))
)

(:action make-product-p215
:parameters ()
:precondition (and (not-made p215)(started o9)(started o33)(started o66)(started o181)(started o196)(started o210)(started o257)(started o265))
:effect (and (not (not-made p215)) (made p215))
)

(:action make-product-p216
:parameters ()
:precondition (and (not-made p216)(started o35)(started o52)(started o74)(started o88)(started o125)(started o198)(started o220)(started o236))
:effect (and (not (not-made p216)) (made p216))
)

(:action make-product-p217
:parameters ()
:precondition (and (not-made p217)(started o2)(started o36)(started o61)(started o109)(started o169)(started o185))
:effect (and (not (not-made p217)) (made p217))
)

(:action make-product-p218
:parameters ()
:precondition (and (not-made p218)(started o11)(started o30)(started o63)(started o66)(started o79)(started o81)(started o150)(started o178)(started o195)(started o275))
:effect (and (not (not-made p218)) (made p218))
)

(:action make-product-p219
:parameters ()
:precondition (and (not-made p219)(started o22)(started o66)(started o94)(started o147)(started o151)(started o175)(started o203)(started o231)(started o234))
:effect (and (not (not-made p219)) (made p219))
)

(:action make-product-p220
:parameters ()
:precondition (and (not-made p220)(started o4)(started o20)(started o151)(started o161)(started o282))
:effect (and (not (not-made p220)) (made p220))
)

(:action make-product-p221
:parameters ()
:precondition (and (not-made p221)(started o36)(started o48)(started o74)(started o107)(started o161)(started o172)(started o176)(started o220))
:effect (and (not (not-made p221)) (made p221))
)

(:action make-product-p222
:parameters ()
:precondition (and (not-made p222)(started o88)(started o102)(started o106)(started o109)(started o124)(started o144)(started o191)(started o204))
:effect (and (not (not-made p222)) (made p222))
)

(:action make-product-p223
:parameters ()
:precondition (and (not-made p223)(started o117)(started o127)(started o214)(started o247)(started o249))
:effect (and (not (not-made p223)) (made p223))
)

(:action make-product-p224
:parameters ()
:precondition (and (not-made p224)(started o23)(started o251)(started o290))
:effect (and (not (not-made p224)) (made p224))
)

(:action make-product-p225
:parameters ()
:precondition (and (not-made p225)(started o16)(started o27)(started o64)(started o211)(started o237))
:effect (and (not (not-made p225)) (made p225))
)

(:action make-product-p226
:parameters ()
:precondition (and (not-made p226)(started o30)(started o66)(started o68)(started o93)(started o106)(started o161)(started o183)(started o233))
:effect (and (not (not-made p226)) (made p226))
)

(:action make-product-p227
:parameters ()
:precondition (and (not-made p227)(started o45)(started o56)(started o63)(started o127)(started o175)(started o183)(started o221)(started o232)(started o273))
:effect (and (not (not-made p227)) (made p227))
)

(:action make-product-p228
:parameters ()
:precondition (and (not-made p228)(started o60)(started o103)(started o114)(started o131)(started o161)(started o202)(started o234)(started o257))
:effect (and (not (not-made p228)) (made p228))
)

(:action make-product-p229
:parameters ()
:precondition (and (not-made p229)(started o34)(started o45)(started o94)(started o105)(started o114)(started o132)(started o142)(started o146)(started o166)(started o175))
:effect (and (not (not-made p229)) (made p229))
)

(:action make-product-p230
:parameters ()
:precondition (and (not-made p230)(started o8)(started o32)(started o50)(started o95)(started o97)(started o109)(started o154)(started o195)(started o245))
:effect (and (not (not-made p230)) (made p230))
)

(:action make-product-p231
:parameters ()
:precondition (and (not-made p231)(started o50)(started o75)(started o214))
:effect (and (not (not-made p231)) (made p231))
)

(:action make-product-p232
:parameters ()
:precondition (and (not-made p232)(started o84)(started o95)(started o167)(started o180)(started o202)(started o244)(started o267))
:effect (and (not (not-made p232)) (made p232))
)

(:action make-product-p233
:parameters ()
:precondition (and (not-made p233)(started o10)(started o65)(started o83)(started o114)(started o118)(started o130)(started o224))
:effect (and (not (not-made p233)) (made p233))
)

(:action make-product-p234
:parameters ()
:precondition (and (not-made p234)(started o9)(started o45)(started o124)(started o180)(started o186)(started o189)(started o190)(started o200)(started o212))
:effect (and (not (not-made p234)) (made p234))
)

(:action make-product-p235
:parameters ()
:precondition (and (not-made p235)(started o97)(started o111)(started o154)(started o206)(started o223)(started o224))
:effect (and (not (not-made p235)) (made p235))
)

(:action make-product-p236
:parameters ()
:precondition (and (not-made p236)(started o29)(started o33)(started o92)(started o94)(started o125)(started o131)(started o144)(started o171))
:effect (and (not (not-made p236)) (made p236))
)

(:action make-product-p237
:parameters ()
:precondition (and (not-made p237)(started o4)(started o19)(started o192)(started o274)(started o287))
:effect (and (not (not-made p237)) (made p237))
)

(:action make-product-p238
:parameters ()
:precondition (and (not-made p238)(started o56)(started o75))
:effect (and (not (not-made p238)) (made p238))
)

(:action make-product-p239
:parameters ()
:precondition (and (not-made p239)(started o34)(started o52)(started o54)(started o61)(started o65)(started o94)(started o111)(started o142)(started o195)(started o199)(started o248))
:effect (and (not (not-made p239)) (made p239))
)

(:action make-product-p240
:parameters ()
:precondition (and (not-made p240)(started o40)(started o101)(started o206)(started o254))
:effect (and (not (not-made p240)) (made p240))
)

(:action make-product-p241
:parameters ()
:precondition (and (not-made p241)(started o5)(started o33)(started o36)(started o39)(started o73)(started o149)(started o153)(started o176)(started o178)(started o198)(started o200)(started o267))
:effect (and (not (not-made p241)) (made p241))
)

(:action make-product-p242
:parameters ()
:precondition (and (not-made p242)(started o6)(started o8)(started o48)(started o149)(started o167)(started o214)(started o238)(started o251)(started o254))
:effect (and (not (not-made p242)) (made p242))
)

(:action make-product-p243
:parameters ()
:precondition (and (not-made p243)(started o58)(started o135)(started o179)(started o229)(started o231)(started o275))
:effect (and (not (not-made p243)) (made p243))
)

(:action make-product-p244
:parameters ()
:precondition (and (not-made p244)(started o160)(started o203)(started o208)(started o219)(started o288))
:effect (and (not (not-made p244)) (made p244))
)

(:action make-product-p245
:parameters ()
:precondition (and (not-made p245)(started o38)(started o110)(started o119)(started o126)(started o133)(started o151)(started o226)(started o288))
:effect (and (not (not-made p245)) (made p245))
)

(:action make-product-p246
:parameters ()
:precondition (and (not-made p246)(started o61)(started o73)(started o96)(started o104)(started o130)(started o193)(started o195)(started o248)(started o252)(started o276))
:effect (and (not (not-made p246)) (made p246))
)

(:action make-product-p247
:parameters ()
:precondition (and (not-made p247)(started o13)(started o77)(started o145)(started o182)(started o232)(started o247)(started o283))
:effect (and (not (not-made p247)) (made p247))
)

(:action make-product-p248
:parameters ()
:precondition (and (not-made p248)(started o49)(started o103)(started o106)(started o182)(started o278)(started o284))
:effect (and (not (not-made p248)) (made p248))
)

(:action make-product-p249
:parameters ()
:precondition (and (not-made p249)(started o9)(started o21)(started o70)(started o176)(started o195)(started o248))
:effect (and (not (not-made p249)) (made p249))
)

(:action make-product-p250
:parameters ()
:precondition (and (not-made p250)(started o79)(started o95)(started o196))
:effect (and (not (not-made p250)) (made p250))
)

(:action make-product-p251
:parameters ()
:precondition (and (not-made p251)(started o124)(started o160)(started o183)(started o253))
:effect (and (not (not-made p251)) (made p251))
)

(:action make-product-p252
:parameters ()
:precondition (and (not-made p252)(started o39)(started o93)(started o265)(started o270)(started o283)(started o285)(started o289))
:effect (and (not (not-made p252)) (made p252))
)

(:action make-product-p253
:parameters ()
:precondition (and (not-made p253)(started o25)(started o83)(started o108)(started o155)(started o171)(started o237)(started o267)(started o278))
:effect (and (not (not-made p253)) (made p253))
)

(:action make-product-p254
:parameters ()
:precondition (and (not-made p254)(started o11)(started o22)(started o25)(started o47)(started o92)(started o102)(started o130)(started o163)(started o169)(started o177)(started o234)(started o245)(started o251))
:effect (and (not (not-made p254)) (made p254))
)

(:action make-product-p255
:parameters ()
:precondition (and (not-made p255)(started o11)(started o28)(started o68)(started o130)(started o165)(started o264)(started o282)(started o286)(started o287))
:effect (and (not (not-made p255)) (made p255))
)

(:action make-product-p256
:parameters ()
:precondition (and (not-made p256)(started o10)(started o123)(started o146)(started o152)(started o178)(started o179)(started o180)(started o217)(started o232)(started o241)(started o288))
:effect (and (not (not-made p256)) (made p256))
)

(:action make-product-p257
:parameters ()
:precondition (and (not-made p257)(started o6)(started o48)(started o74)(started o80)(started o112)(started o149)(started o183)(started o193))
:effect (and (not (not-made p257)) (made p257))
)

(:action make-product-p258
:parameters ()
:precondition (and (not-made p258)(started o8)(started o34)(started o197)(started o276))
:effect (and (not (not-made p258)) (made p258))
)

(:action make-product-p259
:parameters ()
:precondition (and (not-made p259)(started o79)(started o152)(started o172)(started o244))
:effect (and (not (not-made p259)) (made p259))
)

(:action make-product-p260
:parameters ()
:precondition (and (not-made p260)(started o11)(started o16)(started o36)(started o54)(started o83)(started o120)(started o131)(started o158)(started o186)(started o195)(started o234)(started o260))
:effect (and (not (not-made p260)) (made p260))
)

(:action make-product-p261
:parameters ()
:precondition (and (not-made p261)(started o30)(started o50)(started o70)(started o88)(started o110)(started o111)(started o142)(started o181)(started o195)(started o207))
:effect (and (not (not-made p261)) (made p261))
)

(:action make-product-p262
:parameters ()
:precondition (and (not-made p262)(started o113)(started o131)(started o134)(started o144)(started o163)(started o193)(started o215)(started o242)(started o283))
:effect (and (not (not-made p262)) (made p262))
)

(:action make-product-p263
:parameters ()
:precondition (and (not-made p263)(started o127)(started o141)(started o198))
:effect (and (not (not-made p263)) (made p263))
)

(:action make-product-p264
:parameters ()
:precondition (and (not-made p264)(started o14)(started o26)(started o104)(started o162)(started o165)(started o187)(started o199)(started o261))
:effect (and (not (not-made p264)) (made p264))
)

(:action make-product-p265
:parameters ()
:precondition (and (not-made p265)(started o1)(started o71)(started o123)(started o144)(started o223)(started o262))
:effect (and (not (not-made p265)) (made p265))
)

(:action make-product-p266
:parameters ()
:precondition (and (not-made p266)(started o59)(started o106)(started o107)(started o225)(started o264)(started o270))
:effect (and (not (not-made p266)) (made p266))
)

(:action make-product-p267
:parameters ()
:precondition (and (not-made p267)(started o6)(started o74)(started o128)(started o129)(started o181)(started o188)(started o200)(started o287))
:effect (and (not (not-made p267)) (made p267))
)

(:action make-product-p268
:parameters ()
:precondition (and (not-made p268)(started o120)(started o144)(started o227)(started o289))
:effect (and (not (not-made p268)) (made p268))
)

(:action make-product-p269
:parameters ()
:precondition (and (not-made p269)(started o9)(started o14)(started o80)(started o86)(started o168)(started o206)(started o263))
:effect (and (not (not-made p269)) (made p269))
)

(:action make-product-p270
:parameters ()
:precondition (and (not-made p270)(started o40)(started o89)(started o105)(started o164)(started o196)(started o241)(started o264))
:effect (and (not (not-made p270)) (made p270))
)

(:action make-product-p271
:parameters ()
:precondition (and (not-made p271)(started o42)(started o72)(started o187)(started o251)(started o282))
:effect (and (not (not-made p271)) (made p271))
)

(:action make-product-p272
:parameters ()
:precondition (and (not-made p272)(started o11)(started o25)(started o61)(started o71)(started o81)(started o218)(started o220)(started o241)(started o242))
:effect (and (not (not-made p272)) (made p272))
)

(:action make-product-p273
:parameters ()
:precondition (and (not-made p273)(started o46)(started o83)(started o279))
:effect (and (not (not-made p273)) (made p273))
)

(:action make-product-p274
:parameters ()
:precondition (and (not-made p274)(started o1)(started o18)(started o45)(started o164)(started o198)(started o216)(started o224)(started o235)(started o247)(started o252)(started o267)(started o277))
:effect (and (not (not-made p274)) (made p274))
)

(:action make-product-p275
:parameters ()
:precondition (and (not-made p275)(started o10)(started o81)(started o200)(started o209))
:effect (and (not (not-made p275)) (made p275))
)

(:action make-product-p276
:parameters ()
:precondition (and (not-made p276)(started o24)(started o42)(started o74)(started o97)(started o189)(started o196)(started o225)(started o235))
:effect (and (not (not-made p276)) (made p276))
)

(:action make-product-p277
:parameters ()
:precondition (and (not-made p277)(started o25)(started o38)(started o78)(started o102)(started o126)(started o139)(started o141)(started o172)(started o201)(started o248)(started o280))
:effect (and (not (not-made p277)) (made p277))
)

(:action make-product-p278
:parameters ()
:precondition (and (not-made p278)(started o43)(started o167)(started o173)(started o176)(started o238)(started o239)(started o265)(started o266))
:effect (and (not (not-made p278)) (made p278))
)

(:action make-product-p279
:parameters ()
:precondition (and (not-made p279)(started o10)(started o64)(started o67)(started o127)(started o199)(started o235)(started o241))
:effect (and (not (not-made p279)) (made p279))
)

(:action make-product-p280
:parameters ()
:precondition (and (not-made p280)(started o13)(started o39)(started o52)(started o60)(started o83)(started o96)(started o176)(started o217)(started o263))
:effect (and (not (not-made p280)) (made p280))
)

(:action make-product-p281
:parameters ()
:precondition (and (not-made p281)(started o89)(started o111)(started o121)(started o164)(started o221)(started o225)(started o227)(started o232)(started o233))
:effect (and (not (not-made p281)) (made p281))
)

(:action make-product-p282
:parameters ()
:precondition (and (not-made p282)(started o127)(started o141)(started o220))
:effect (and (not (not-made p282)) (made p282))
)

(:action make-product-p283
:parameters ()
:precondition (and (not-made p283)(started o2)(started o23)(started o29)(started o84)(started o97)(started o189)(started o198)(started o263))
:effect (and (not (not-made p283)) (made p283))
)

(:action make-product-p284
:parameters ()
:precondition (and (not-made p284)(started o43)(started o99)(started o146))
:effect (and (not (not-made p284)) (made p284))
)

(:action make-product-p285
:parameters ()
:precondition (and (not-made p285)(started o77)(started o85)(started o142)(started o148)(started o151)(started o168)(started o205))
:effect (and (not (not-made p285)) (made p285))
)

(:action make-product-p286
:parameters ()
:precondition (and (not-made p286)(started o3)(started o79)(started o123)(started o181)(started o239)(started o250)(started o277))
:effect (and (not (not-made p286)) (made p286))
)

(:action make-product-p287
:parameters ()
:precondition (and (not-made p287)(started o19)(started o29)(started o43)(started o46)(started o84)(started o97)(started o114)(started o148)(started o215)(started o282)(started o285))
:effect (and (not (not-made p287)) (made p287))
)

(:action make-product-p288
:parameters ()
:precondition (and (not-made p288)(started o4)(started o8)(started o10)(started o41)(started o70)(started o154)(started o195)(started o263)(started o289))
:effect (and (not (not-made p288)) (made p288))
)

(:action make-product-p289
:parameters ()
:precondition (and (not-made p289)(started o4)(started o78)(started o132)(started o179)(started o280))
:effect (and (not (not-made p289)) (made p289))
)

(:action make-product-p290
:parameters ()
:precondition (and (not-made p290)(started o28)(started o55)(started o75)(started o92)(started o166)(started o245))
:effect (and (not (not-made p290)) (made p290))
)

(:action ship-order-o1
:parameters (?avail ?new-avail - count)
:precondition (and (started o1)(made p52)(made p73)(made p84)(made p94)(made p101)(made p142)(made p160)(made p185)(made p265)(made p274)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o1))(shipped o1)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o2
:parameters (?avail ?new-avail - count)
:precondition (and (started o2)(made p103)(made p111)(made p169)(made p191)(made p217)(made p283)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o2))(shipped o2)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o3
:parameters (?avail ?new-avail - count)
:precondition (and (started o3)(made p11)(made p50)(made p104)(made p136)(made p174)(made p175)(made p214)(made p286)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o3))(shipped o3)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o4
:parameters (?avail ?new-avail - count)
:precondition (and (started o4)(made p41)(made p64)(made p105)(made p143)(made p220)(made p237)(made p288)(made p289)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o4))(shipped o4)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o5
:parameters (?avail ?new-avail - count)
:precondition (and (started o5)(made p68)(made p69)(made p72)(made p111)(made p152)(made p191)(made p210)(made p241)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o5))(shipped o5)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o6
:parameters (?avail ?new-avail - count)
:precondition (and (started o6)(made p15)(made p23)(made p81)(made p105)(made p155)(made p212)(made p242)(made p257)(made p267)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o6))(shipped o6)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o7
:parameters (?avail ?new-avail - count)
:precondition (and (started o7)(made p32)(made p54)(made p61)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o7))(shipped o7)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o8
:parameters (?avail ?new-avail - count)
:precondition (and (started o8)(made p20)(made p61)(made p92)(made p130)(made p198)(made p230)(made p242)(made p258)(made p288)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o8))(shipped o8)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o9
:parameters (?avail ?new-avail - count)
:precondition (and (started o9)(made p18)(made p70)(made p86)(made p98)(made p101)(made p146)(made p155)(made p170)(made p215)(made p234)(made p249)(made p269)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o9))(shipped o9)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o10
:parameters (?avail ?new-avail - count)
:precondition (and (started o10)(made p13)(made p17)(made p40)(made p93)(made p154)(made p165)(made p177)(made p184)(made p233)(made p256)(made p275)(made p279)(made p288)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o10))(shipped o10)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o11
:parameters (?avail ?new-avail - count)
:precondition (and (started o11)(made p39)(made p42)(made p186)(made p193)(made p205)(made p218)(made p254)(made p255)(made p260)(made p272)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o11))(shipped o11)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o12
:parameters (?avail ?new-avail - count)
:precondition (and (started o12)(made p18)(made p79)(made p126)(made p144)(made p213)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o12))(shipped o12)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o13
:parameters (?avail ?new-avail - count)
:precondition (and (started o13)(made p37)(made p141)(made p173)(made p196)(made p247)(made p280)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o13))(shipped o13)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o14
:parameters (?avail ?new-avail - count)
:precondition (and (started o14)(made p58)(made p64)(made p102)(made p108)(made p120)(made p130)(made p264)(made p269)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o14))(shipped o14)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o15
:parameters (?avail ?new-avail - count)
:precondition (and (started o15)(made p70)(made p140)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o15))(shipped o15)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o16
:parameters (?avail ?new-avail - count)
:precondition (and (started o16)(made p6)(made p63)(made p104)(made p142)(made p225)(made p260)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o16))(shipped o16)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o17
:parameters (?avail ?new-avail - count)
:precondition (and (started o17)(made p9)(made p134)(made p156)(made p205)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o17))(shipped o17)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o18
:parameters (?avail ?new-avail - count)
:precondition (and (started o18)(made p36)(made p53)(made p112)(made p115)(made p129)(made p158)(made p162)(made p169)(made p179)(made p274)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o18))(shipped o18)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o19
:parameters (?avail ?new-avail - count)
:precondition (and (started o19)(made p150)(made p162)(made p237)(made p287)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o19))(shipped o19)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o20
:parameters (?avail ?new-avail - count)
:precondition (and (started o20)(made p30)(made p71)(made p97)(made p220)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o20))(shipped o20)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o21
:parameters (?avail ?new-avail - count)
:precondition (and (started o21)(made p88)(made p95)(made p151)(made p171)(made p177)(made p202)(made p249)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o21))(shipped o21)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o22
:parameters (?avail ?new-avail - count)
:precondition (and (started o22)(made p2)(made p63)(made p67)(made p73)(made p100)(made p111)(made p179)(made p190)(made p193)(made p202)(made p219)(made p254)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o22))(shipped o22)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o23
:parameters (?avail ?new-avail - count)
:precondition (and (started o23)(made p29)(made p49)(made p123)(made p153)(made p224)(made p283)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o23))(shipped o23)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o24
:parameters (?avail ?new-avail - count)
:precondition (and (started o24)(made p1)(made p8)(made p47)(made p176)(made p211)(made p276)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o24))(shipped o24)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o25
:parameters (?avail ?new-avail - count)
:precondition (and (started o25)(made p43)(made p48)(made p71)(made p77)(made p107)(made p139)(made p195)(made p253)(made p254)(made p272)(made p277)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o25))(shipped o25)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o26
:parameters (?avail ?new-avail - count)
:precondition (and (started o26)(made p14)(made p39)(made p57)(made p73)(made p122)(made p129)(made p189)(made p190)(made p191)(made p264)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o26))(shipped o26)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o27
:parameters (?avail ?new-avail - count)
:precondition (and (started o27)(made p55)(made p119)(made p140)(made p141)(made p175)(made p225)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o27))(shipped o27)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o28
:parameters (?avail ?new-avail - count)
:precondition (and (started o28)(made p73)(made p88)(made p255)(made p290)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o28))(shipped o28)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o29
:parameters (?avail ?new-avail - count)
:precondition (and (started o29)(made p102)(made p128)(made p168)(made p187)(made p236)(made p283)(made p287)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o29))(shipped o29)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o30
:parameters (?avail ?new-avail - count)
:precondition (and (started o30)(made p71)(made p73)(made p138)(made p148)(made p202)(made p218)(made p226)(made p261)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o30))(shipped o30)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o31
:parameters (?avail ?new-avail - count)
:precondition (and (started o31)(made p55)(made p63)(made p119)(made p176)(made p199)(made p200)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o31))(shipped o31)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o32
:parameters (?avail ?new-avail - count)
:precondition (and (started o32)(made p19)(made p33)(made p54)(made p65)(made p101)(made p170)(made p230)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o32))(shipped o32)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o33
:parameters (?avail ?new-avail - count)
:precondition (and (started o33)(made p28)(made p70)(made p79)(made p103)(made p156)(made p173)(made p183)(made p198)(made p215)(made p236)(made p241)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o33))(shipped o33)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o34
:parameters (?avail ?new-avail - count)
:precondition (and (started o34)(made p76)(made p167)(made p183)(made p229)(made p239)(made p258)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o34))(shipped o34)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o35
:parameters (?avail ?new-avail - count)
:precondition (and (started o35)(made p74)(made p104)(made p203)(made p216)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o35))(shipped o35)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o36
:parameters (?avail ?new-avail - count)
:precondition (and (started o36)(made p10)(made p53)(made p60)(made p72)(made p85)(made p103)(made p104)(made p145)(made p162)(made p178)(made p203)(made p206)(made p217)(made p221)(made p241)(made p260)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o36))(shipped o36)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o37
:parameters (?avail ?new-avail - count)
:precondition (and (started o37)(made p10)(made p194)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o37))(shipped o37)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o38
:parameters (?avail ?new-avail - count)
:precondition (and (started o38)(made p78)(made p82)(made p152)(made p179)(made p245)(made p277)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o38))(shipped o38)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o39
:parameters (?avail ?new-avail - count)
:precondition (and (started o39)(made p9)(made p104)(made p109)(made p126)(made p241)(made p252)(made p280)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o39))(shipped o39)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o40
:parameters (?avail ?new-avail - count)
:precondition (and (started o40)(made p1)(made p38)(made p45)(made p80)(made p159)(made p165)(made p181)(made p200)(made p240)(made p270)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o40))(shipped o40)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o41
:parameters (?avail ?new-avail - count)
:precondition (and (started o41)(made p18)(made p27)(made p86)(made p101)(made p152)(made p189)(made p190)(made p288)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o41))(shipped o41)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o42
:parameters (?avail ?new-avail - count)
:precondition (and (started o42)(made p9)(made p44)(made p111)(made p133)(made p135)(made p149)(made p208)(made p271)(made p276)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o42))(shipped o42)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o43
:parameters (?avail ?new-avail - count)
:precondition (and (started o43)(made p99)(made p102)(made p156)(made p278)(made p284)(made p287)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o43))(shipped o43)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o44
:parameters (?avail ?new-avail - count)
:precondition (and (started o44)(made p15)(made p68)(made p195)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o44))(shipped o44)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o45
:parameters (?avail ?new-avail - count)
:precondition (and (started o45)(made p70)(made p91)(made p101)(made p129)(made p196)(made p197)(made p227)(made p229)(made p234)(made p274)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o45))(shipped o45)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o46
:parameters (?avail ?new-avail - count)
:precondition (and (started o46)(made p114)(made p152)(made p163)(made p182)(made p186)(made p273)(made p287)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o46))(shipped o46)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o47
:parameters (?avail ?new-avail - count)
:precondition (and (started o47)(made p1)(made p6)(made p45)(made p73)(made p193)(made p254)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o47))(shipped o47)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o48
:parameters (?avail ?new-avail - count)
:precondition (and (started o48)(made p48)(made p112)(made p151)(made p172)(made p175)(made p190)(made p206)(made p221)(made p242)(made p257)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o48))(shipped o48)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o49
:parameters (?avail ?new-avail - count)
:precondition (and (started o49)(made p35)(made p38)(made p95)(made p173)(made p194)(made p198)(made p199)(made p248)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o49))(shipped o49)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o50
:parameters (?avail ?new-avail - count)
:precondition (and (started o50)(made p2)(made p188)(made p189)(made p230)(made p231)(made p261)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o50))(shipped o50)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o51
:parameters (?avail ?new-avail - count)
:precondition (and (started o51)(made p7)(made p78)(made p85)(made p98)(made p107)(made p115)(made p136)(made p150)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o51))(shipped o51)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o52
:parameters (?avail ?new-avail - count)
:precondition (and (started o52)(made p24)(made p57)(made p142)(made p216)(made p239)(made p280)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o52))(shipped o52)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o53
:parameters (?avail ?new-avail - count)
:precondition (and (started o53)(made p63)(made p101)(made p108)(made p115)(made p133)(made p155)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o53))(shipped o53)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o54
:parameters (?avail ?new-avail - count)
:precondition (and (started o54)(made p2)(made p38)(made p43)(made p50)(made p58)(made p73)(made p85)(made p116)(made p181)(made p214)(made p239)(made p260)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o54))(shipped o54)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o55
:parameters (?avail ?new-avail - count)
:precondition (and (started o55)(made p35)(made p113)(made p290)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o55))(shipped o55)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o56
:parameters (?avail ?new-avail - count)
:precondition (and (started o56)(made p5)(made p20)(made p37)(made p68)(made p91)(made p135)(made p204)(made p227)(made p238)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o56))(shipped o56)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o57
:parameters (?avail ?new-avail - count)
:precondition (and (started o57)(made p33)(made p51)(made p102)(made p134)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o57))(shipped o57)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o58
:parameters (?avail ?new-avail - count)
:precondition (and (started o58)(made p155)(made p164)(made p165)(made p198)(made p243)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o58))(shipped o58)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o59
:parameters (?avail ?new-avail - count)
:precondition (and (started o59)(made p29)(made p266)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o59))(shipped o59)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o60
:parameters (?avail ?new-avail - count)
:precondition (and (started o60)(made p63)(made p74)(made p82)(made p83)(made p96)(made p126)(made p127)(made p204)(made p228)(made p280)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o60))(shipped o60)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o61
:parameters (?avail ?new-avail - count)
:precondition (and (started o61)(made p43)(made p47)(made p52)(made p152)(made p189)(made p217)(made p239)(made p246)(made p272)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o61))(shipped o61)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o62
:parameters (?avail ?new-avail - count)
:precondition (and (started o62)(made p22)(made p171)(made p200)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o62))(shipped o62)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o63
:parameters (?avail ?new-avail - count)
:precondition (and (started o63)(made p21)(made p120)(made p128)(made p139)(made p146)(made p192)(made p211)(made p218)(made p227)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o63))(shipped o63)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o64
:parameters (?avail ?new-avail - count)
:precondition (and (started o64)(made p34)(made p115)(made p148)(made p225)(made p279)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o64))(shipped o64)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o65
:parameters (?avail ?new-avail - count)
:precondition (and (started o65)(made p2)(made p52)(made p133)(made p165)(made p177)(made p186)(made p233)(made p239)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o65))(shipped o65)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o66
:parameters (?avail ?new-avail - count)
:precondition (and (started o66)(made p20)(made p59)(made p60)(made p65)(made p88)(made p212)(made p215)(made p218)(made p219)(made p226)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o66))(shipped o66)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o67
:parameters (?avail ?new-avail - count)
:precondition (and (started o67)(made p21)(made p46)(made p52)(made p80)(made p130)(made p165)(made p171)(made p279)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o67))(shipped o67)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o68
:parameters (?avail ?new-avail - count)
:precondition (and (started o68)(made p5)(made p47)(made p62)(made p79)(made p154)(made p226)(made p255)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o68))(shipped o68)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o69
:parameters (?avail ?new-avail - count)
:precondition (and (started o69)(made p2)(made p28)(made p31)(made p81)(made p195)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o69))(shipped o69)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o70
:parameters (?avail ?new-avail - count)
:precondition (and (started o70)(made p88)(made p94)(made p249)(made p261)(made p288)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o70))(shipped o70)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o71
:parameters (?avail ?new-avail - count)
:precondition (and (started o71)(made p25)(made p42)(made p43)(made p44)(made p79)(made p104)(made p178)(made p190)(made p194)(made p265)(made p272)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o71))(shipped o71)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o72
:parameters (?avail ?new-avail - count)
:precondition (and (started o72)(made p33)(made p79)(made p87)(made p143)(made p150)(made p156)(made p181)(made p271)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o72))(shipped o72)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o73
:parameters (?avail ?new-avail - count)
:precondition (and (started o73)(made p35)(made p55)(made p112)(made p113)(made p115)(made p126)(made p133)(made p196)(made p241)(made p246)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o73))(shipped o73)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o74
:parameters (?avail ?new-avail - count)
:precondition (and (started o74)(made p43)(made p54)(made p86)(made p115)(made p190)(made p192)(made p198)(made p216)(made p221)(made p257)(made p267)(made p276)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o74))(shipped o74)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o75
:parameters (?avail ?new-avail - count)
:precondition (and (started o75)(made p52)(made p209)(made p231)(made p238)(made p290)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o75))(shipped o75)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o76
:parameters (?avail ?new-avail - count)
:precondition (and (started o76)(made p16)(made p74)(made p178)(made p182)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o76))(shipped o76)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o77
:parameters (?avail ?new-avail - count)
:precondition (and (started o77)(made p42)(made p60)(made p100)(made p162)(made p189)(made p193)(made p197)(made p213)(made p247)(made p285)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o77))(shipped o77)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o78
:parameters (?avail ?new-avail - count)
:precondition (and (started o78)(made p2)(made p32)(made p44)(made p56)(made p146)(made p151)(made p157)(made p277)(made p289)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o78))(shipped o78)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o79
:parameters (?avail ?new-avail - count)
:precondition (and (started o79)(made p45)(made p96)(made p136)(made p157)(made p218)(made p250)(made p259)(made p286)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o79))(shipped o79)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o80
:parameters (?avail ?new-avail - count)
:precondition (and (started o80)(made p31)(made p33)(made p149)(made p159)(made p191)(made p257)(made p269)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o80))(shipped o80)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o81
:parameters (?avail ?new-avail - count)
:precondition (and (started o81)(made p26)(made p34)(made p60)(made p72)(made p94)(made p106)(made p127)(made p136)(made p218)(made p272)(made p275)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o81))(shipped o81)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o82
:parameters (?avail ?new-avail - count)
:precondition (and (started o82)(made p34)(made p85)(made p199)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o82))(shipped o82)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o83
:parameters (?avail ?new-avail - count)
:precondition (and (started o83)(made p8)(made p35)(made p76)(made p123)(made p138)(made p147)(made p175)(made p233)(made p253)(made p260)(made p273)(made p280)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o83))(shipped o83)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o84
:parameters (?avail ?new-avail - count)
:precondition (and (started o84)(made p30)(made p40)(made p105)(made p232)(made p283)(made p287)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o84))(shipped o84)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o85
:parameters (?avail ?new-avail - count)
:precondition (and (started o85)(made p75)(made p132)(made p285)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o85))(shipped o85)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o86
:parameters (?avail ?new-avail - count)
:precondition (and (started o86)(made p96)(made p110)(made p269)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o86))(shipped o86)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o87
:parameters (?avail ?new-avail - count)
:precondition (and (started o87)(made p55)(made p93)(made p176)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o87))(shipped o87)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o88
:parameters (?avail ?new-avail - count)
:precondition (and (started o88)(made p2)(made p40)(made p89)(made p159)(made p161)(made p188)(made p216)(made p222)(made p261)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o88))(shipped o88)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o89
:parameters (?avail ?new-avail - count)
:precondition (and (started o89)(made p99)(made p153)(made p165)(made p270)(made p281)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o89))(shipped o89)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o90
:parameters (?avail ?new-avail - count)
:precondition (and (started o90)(made p47)(made p79)(made p84)(made p123)(made p147)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o90))(shipped o90)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o91
:parameters (?avail ?new-avail - count)
:precondition (and (started o91)(made p60)(made p162)(made p167)(made p184)(made p195)(made p206)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o91))(shipped o91)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o92
:parameters (?avail ?new-avail - count)
:precondition (and (started o92)(made p14)(made p45)(made p47)(made p134)(made p236)(made p254)(made p290)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o92))(shipped o92)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o93
:parameters (?avail ?new-avail - count)
:precondition (and (started o93)(made p82)(made p136)(made p180)(made p197)(made p202)(made p226)(made p252)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o93))(shipped o93)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o94
:parameters (?avail ?new-avail - count)
:precondition (and (started o94)(made p43)(made p54)(made p80)(made p136)(made p167)(made p219)(made p229)(made p236)(made p239)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o94))(shipped o94)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o95
:parameters (?avail ?new-avail - count)
:precondition (and (started o95)(made p24)(made p74)(made p155)(made p177)(made p185)(made p193)(made p206)(made p230)(made p232)(made p250)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o95))(shipped o95)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o96
:parameters (?avail ?new-avail - count)
:precondition (and (started o96)(made p28)(made p102)(made p126)(made p195)(made p208)(made p246)(made p280)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o96))(shipped o96)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o97
:parameters (?avail ?new-avail - count)
:precondition (and (started o97)(made p8)(made p56)(made p105)(made p197)(made p230)(made p235)(made p276)(made p283)(made p287)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o97))(shipped o97)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o98
:parameters (?avail ?new-avail - count)
:precondition (and (started o98)(made p47)(made p103)(made p148)(made p164)(made p179)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o98))(shipped o98)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o99
:parameters (?avail ?new-avail - count)
:precondition (and (started o99)(made p18)(made p27)(made p28)(made p95)(made p170)(made p180)(made p284)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o99))(shipped o99)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o100
:parameters (?avail ?new-avail - count)
:precondition (and (started o100)(made p4)(made p63)(made p87)(made p131)(made p211)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o100))(shipped o100)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o101
:parameters (?avail ?new-avail - count)
:precondition (and (started o101)(made p11)(made p13)(made p19)(made p32)(made p40)(made p46)(made p57)(made p135)(made p207)(made p240)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o101))(shipped o101)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o102
:parameters (?avail ?new-avail - count)
:precondition (and (started o102)(made p21)(made p63)(made p70)(made p107)(made p108)(made p109)(made p122)(made p222)(made p254)(made p277)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o102))(shipped o102)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o103
:parameters (?avail ?new-avail - count)
:precondition (and (started o103)(made p54)(made p70)(made p87)(made p97)(made p109)(made p140)(made p228)(made p248)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o103))(shipped o103)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o104
:parameters (?avail ?new-avail - count)
:precondition (and (started o104)(made p25)(made p97)(made p132)(made p143)(made p199)(made p246)(made p264)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o104))(shipped o104)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o105
:parameters (?avail ?new-avail - count)
:precondition (and (started o105)(made p89)(made p96)(made p130)(made p151)(made p229)(made p270)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o105))(shipped o105)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o106
:parameters (?avail ?new-avail - count)
:precondition (and (started o106)(made p14)(made p97)(made p140)(made p168)(made p174)(made p176)(made p201)(made p222)(made p226)(made p248)(made p266)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o106))(shipped o106)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o107
:parameters (?avail ?new-avail - count)
:precondition (and (started o107)(made p16)(made p24)(made p59)(made p100)(made p112)(made p127)(made p168)(made p174)(made p213)(made p221)(made p266)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o107))(shipped o107)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o108
:parameters (?avail ?new-avail - count)
:precondition (and (started o108)(made p5)(made p45)(made p103)(made p105)(made p133)(made p136)(made p185)(made p201)(made p253)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o108))(shipped o108)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o109
:parameters (?avail ?new-avail - count)
:precondition (and (started o109)(made p36)(made p94)(made p109)(made p217)(made p222)(made p230)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o109))(shipped o109)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o110
:parameters (?avail ?new-avail - count)
:precondition (and (started o110)(made p6)(made p92)(made p120)(made p245)(made p261)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o110))(shipped o110)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o111
:parameters (?avail ?new-avail - count)
:precondition (and (started o111)(made p34)(made p58)(made p137)(made p141)(made p197)(made p207)(made p235)(made p239)(made p261)(made p281)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o111))(shipped o111)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o112
:parameters (?avail ?new-avail - count)
:precondition (and (started o112)(made p3)(made p46)(made p257)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o112))(shipped o112)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o113
:parameters (?avail ?new-avail - count)
:precondition (and (started o113)(made p9)(made p25)(made p82)(made p169)(made p173)(made p262)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o113))(shipped o113)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o114
:parameters (?avail ?new-avail - count)
:precondition (and (started o114)(made p19)(made p67)(made p80)(made p96)(made p112)(made p146)(made p153)(made p173)(made p228)(made p229)(made p233)(made p287)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o114))(shipped o114)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o115
:parameters (?avail ?new-avail - count)
:precondition (and (started o115)(made p40)(made p66)(made p69)(made p103)(made p151)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o115))(shipped o115)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o116
:parameters (?avail ?new-avail - count)
:precondition (and (started o116)(made p80)(made p130)(made p141)(made p157)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o116))(shipped o116)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o117
:parameters (?avail ?new-avail - count)
:precondition (and (started o117)(made p13)(made p36)(made p44)(made p112)(made p144)(made p223)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o117))(shipped o117)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o118
:parameters (?avail ?new-avail - count)
:precondition (and (started o118)(made p77)(made p108)(made p128)(made p233)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o118))(shipped o118)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o119
:parameters (?avail ?new-avail - count)
:precondition (and (started o119)(made p49)(made p174)(made p245)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o119))(shipped o119)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o120
:parameters (?avail ?new-avail - count)
:precondition (and (started o120)(made p260)(made p268)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o120))(shipped o120)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o121
:parameters (?avail ?new-avail - count)
:precondition (and (started o121)(made p12)(made p19)(made p65)(made p130)(made p157)(made p210)(made p281)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o121))(shipped o121)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o122
:parameters (?avail ?new-avail - count)
:precondition (and (started o122)(made p75)(made p183)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o122))(shipped o122)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o123
:parameters (?avail ?new-avail - count)
:precondition (and (started o123)(made p92)(made p123)(made p160)(made p171)(made p256)(made p265)(made p286)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o123))(shipped o123)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o124
:parameters (?avail ?new-avail - count)
:precondition (and (started o124)(made p30)(made p90)(made p159)(made p191)(made p222)(made p234)(made p251)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o124))(shipped o124)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o125
:parameters (?avail ?new-avail - count)
:precondition (and (started o125)(made p6)(made p17)(made p200)(made p216)(made p236)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o125))(shipped o125)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o126
:parameters (?avail ?new-avail - count)
:precondition (and (started o126)(made p19)(made p45)(made p80)(made p166)(made p245)(made p277)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o126))(shipped o126)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o127
:parameters (?avail ?new-avail - count)
:precondition (and (started o127)(made p3)(made p75)(made p81)(made p130)(made p144)(made p167)(made p213)(made p223)(made p227)(made p263)(made p279)(made p282)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o127))(shipped o127)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o128
:parameters (?avail ?new-avail - count)
:precondition (and (started o128)(made p58)(made p65)(made p69)(made p204)(made p267)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o128))(shipped o128)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o129
:parameters (?avail ?new-avail - count)
:precondition (and (started o129)(made p2)(made p25)(made p42)(made p110)(made p122)(made p160)(made p182)(made p267)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o129))(shipped o129)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o130
:parameters (?avail ?new-avail - count)
:precondition (and (started o130)(made p14)(made p54)(made p113)(made p159)(made p233)(made p246)(made p254)(made p255)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o130))(shipped o130)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o131
:parameters (?avail ?new-avail - count)
:precondition (and (started o131)(made p72)(made p132)(made p189)(made p202)(made p228)(made p236)(made p260)(made p262)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o131))(shipped o131)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o132
:parameters (?avail ?new-avail - count)
:precondition (and (started o132)(made p38)(made p155)(made p192)(made p197)(made p229)(made p289)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o132))(shipped o132)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o133
:parameters (?avail ?new-avail - count)
:precondition (and (started o133)(made p42)(made p124)(made p190)(made p207)(made p245)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o133))(shipped o133)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o134
:parameters (?avail ?new-avail - count)
:precondition (and (started o134)(made p42)(made p100)(made p104)(made p138)(made p140)(made p157)(made p173)(made p176)(made p194)(made p202)(made p262)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o134))(shipped o134)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o135
:parameters (?avail ?new-avail - count)
:precondition (and (started o135)(made p3)(made p44)(made p49)(made p123)(made p189)(made p243)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o135))(shipped o135)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o136
:parameters (?avail ?new-avail - count)
:precondition (and (started o136)(made p9)(made p24)(made p25)(made p98)(made p110)(made p132)(made p156)(made p175)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o136))(shipped o136)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o137
:parameters (?avail ?new-avail - count)
:precondition (and (started o137)(made p10)(made p45)(made p200)(made p205)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o137))(shipped o137)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o138
:parameters (?avail ?new-avail - count)
:precondition (and (started o138)(made p90)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o138))(shipped o138)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o139
:parameters (?avail ?new-avail - count)
:precondition (and (started o139)(made p39)(made p118)(made p146)(made p152)(made p159)(made p190)(made p209)(made p277)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o139))(shipped o139)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o140
:parameters (?avail ?new-avail - count)
:precondition (and (started o140)(made p8)(made p74)(made p89)(made p122)(made p132)(made p146)(made p160)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o140))(shipped o140)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o141
:parameters (?avail ?new-avail - count)
:precondition (and (started o141)(made p22)(made p53)(made p200)(made p205)(made p263)(made p277)(made p282)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o141))(shipped o141)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o142
:parameters (?avail ?new-avail - count)
:precondition (and (started o142)(made p69)(made p113)(made p119)(made p165)(made p169)(made p186)(made p229)(made p239)(made p261)(made p285)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o142))(shipped o142)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o143
:parameters (?avail ?new-avail - count)
:precondition (and (started o143)(made p17)(made p18)(made p32)(made p76)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o143))(shipped o143)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o144
:parameters (?avail ?new-avail - count)
:precondition (and (started o144)(made p39)(made p108)(made p125)(made p150)(made p160)(made p168)(made p207)(made p222)(made p236)(made p262)(made p265)(made p268)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o144))(shipped o144)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o145
:parameters (?avail ?new-avail - count)
:precondition (and (started o145)(made p69)(made p74)(made p82)(made p85)(made p116)(made p132)(made p208)(made p210)(made p247)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o145))(shipped o145)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o146
:parameters (?avail ?new-avail - count)
:precondition (and (started o146)(made p5)(made p41)(made p79)(made p81)(made p103)(made p186)(made p206)(made p229)(made p256)(made p284)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o146))(shipped o146)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o147
:parameters (?avail ?new-avail - count)
:precondition (and (started o147)(made p32)(made p93)(made p127)(made p132)(made p185)(made p198)(made p219)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o147))(shipped o147)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o148
:parameters (?avail ?new-avail - count)
:precondition (and (started o148)(made p109)(made p144)(made p285)(made p287)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o148))(shipped o148)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o149
:parameters (?avail ?new-avail - count)
:precondition (and (started o149)(made p63)(made p65)(made p144)(made p180)(made p214)(made p241)(made p242)(made p257)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o149))(shipped o149)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o150
:parameters (?avail ?new-avail - count)
:precondition (and (started o150)(made p88)(made p165)(made p218)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o150))(shipped o150)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o151
:parameters (?avail ?new-avail - count)
:precondition (and (started o151)(made p1)(made p66)(made p90)(made p120)(made p208)(made p219)(made p220)(made p245)(made p285)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o151))(shipped o151)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o152
:parameters (?avail ?new-avail - count)
:precondition (and (started o152)(made p24)(made p51)(made p202)(made p256)(made p259)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o152))(shipped o152)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o153
:parameters (?avail ?new-avail - count)
:precondition (and (started o153)(made p21)(made p39)(made p127)(made p241)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o153))(shipped o153)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o154
:parameters (?avail ?new-avail - count)
:precondition (and (started o154)(made p144)(made p159)(made p230)(made p235)(made p288)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o154))(shipped o154)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o155
:parameters (?avail ?new-avail - count)
:precondition (and (started o155)(made p134)(made p160)(made p164)(made p253)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o155))(shipped o155)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o156
:parameters (?avail ?new-avail - count)
:precondition (and (started o156)(made p37)(made p43)(made p119)(made p124)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o156))(shipped o156)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o157
:parameters (?avail ?new-avail - count)
:precondition (and (started o157)(made p29)(made p31)(made p66)(made p114)(made p119)(made p127)(made p204)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o157))(shipped o157)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o158
:parameters (?avail ?new-avail - count)
:precondition (and (started o158)(made p4)(made p9)(made p30)(made p31)(made p36)(made p69)(made p76)(made p110)(made p135)(made p198)(made p260)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o158))(shipped o158)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o159
:parameters (?avail ?new-avail - count)
:precondition (and (started o159)(made p4)(made p38)(made p73)(made p123)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o159))(shipped o159)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o160
:parameters (?avail ?new-avail - count)
:precondition (and (started o160)(made p37)(made p108)(made p130)(made p198)(made p202)(made p244)(made p251)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o160))(shipped o160)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o161
:parameters (?avail ?new-avail - count)
:precondition (and (started o161)(made p34)(made p82)(made p180)(made p191)(made p220)(made p221)(made p226)(made p228)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o161))(shipped o161)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o162
:parameters (?avail ?new-avail - count)
:precondition (and (started o162)(made p13)(made p55)(made p58)(made p63)(made p75)(made p128)(made p137)(made p171)(made p213)(made p264)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o162))(shipped o162)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o163
:parameters (?avail ?new-avail - count)
:precondition (and (started o163)(made p108)(made p254)(made p262)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o163))(shipped o163)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o164
:parameters (?avail ?new-avail - count)
:precondition (and (started o164)(made p10)(made p23)(made p25)(made p45)(made p73)(made p93)(made p104)(made p118)(made p122)(made p198)(made p210)(made p214)(made p270)(made p274)(made p281)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o164))(shipped o164)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o165
:parameters (?avail ?new-avail - count)
:precondition (and (started o165)(made p3)(made p36)(made p78)(made p88)(made p151)(made p165)(made p255)(made p264)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o165))(shipped o165)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o166
:parameters (?avail ?new-avail - count)
:precondition (and (started o166)(made p21)(made p80)(made p159)(made p174)(made p229)(made p290)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o166))(shipped o166)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o167
:parameters (?avail ?new-avail - count)
:precondition (and (started o167)(made p64)(made p70)(made p85)(made p90)(made p113)(made p114)(made p129)(made p232)(made p242)(made p278)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o167))(shipped o167)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o168
:parameters (?avail ?new-avail - count)
:precondition (and (started o168)(made p12)(made p16)(made p20)(made p105)(made p195)(made p269)(made p285)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o168))(shipped o168)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o169
:parameters (?avail ?new-avail - count)
:precondition (and (started o169)(made p23)(made p36)(made p83)(made p85)(made p87)(made p108)(made p147)(made p152)(made p217)(made p254)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o169))(shipped o169)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o170
:parameters (?avail ?new-avail - count)
:precondition (and (started o170)(made p35)(made p110)(made p157)(made p162)(made p197)(made p204)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o170))(shipped o170)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o171
:parameters (?avail ?new-avail - count)
:precondition (and (started o171)(made p236)(made p253)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o171))(shipped o171)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o172
:parameters (?avail ?new-avail - count)
:precondition (and (started o172)(made p13)(made p17)(made p131)(made p207)(made p221)(made p259)(made p277)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o172))(shipped o172)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o173
:parameters (?avail ?new-avail - count)
:precondition (and (started o173)(made p3)(made p6)(made p84)(made p144)(made p200)(made p278)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o173))(shipped o173)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o174
:parameters (?avail ?new-avail - count)
:precondition (and (started o174)(made p61)(made p98)(made p135)(made p145)(made p146)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o174))(shipped o174)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o175
:parameters (?avail ?new-avail - count)
:precondition (and (started o175)(made p13)(made p14)(made p21)(made p101)(made p108)(made p181)(made p219)(made p227)(made p229)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o175))(shipped o175)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o176
:parameters (?avail ?new-avail - count)
:precondition (and (started o176)(made p16)(made p59)(made p74)(made p97)(made p118)(made p138)(made p143)(made p179)(made p221)(made p241)(made p249)(made p278)(made p280)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o176))(shipped o176)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o177
:parameters (?avail ?new-avail - count)
:precondition (and (started o177)(made p108)(made p112)(made p145)(made p174)(made p189)(made p254)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o177))(shipped o177)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o178
:parameters (?avail ?new-avail - count)
:precondition (and (started o178)(made p16)(made p49)(made p68)(made p154)(made p199)(made p218)(made p241)(made p256)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o178))(shipped o178)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o179
:parameters (?avail ?new-avail - count)
:precondition (and (started o179)(made p37)(made p82)(made p213)(made p243)(made p256)(made p289)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o179))(shipped o179)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o180
:parameters (?avail ?new-avail - count)
:precondition (and (started o180)(made p31)(made p60)(made p67)(made p69)(made p165)(made p232)(made p234)(made p256)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o180))(shipped o180)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o181
:parameters (?avail ?new-avail - count)
:precondition (and (started o181)(made p19)(made p56)(made p71)(made p131)(made p135)(made p215)(made p261)(made p267)(made p286)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o181))(shipped o181)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o182
:parameters (?avail ?new-avail - count)
:precondition (and (started o182)(made p31)(made p59)(made p80)(made p82)(made p118)(made p153)(made p157)(made p176)(made p247)(made p248)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o182))(shipped o182)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o183
:parameters (?avail ?new-avail - count)
:precondition (and (started o183)(made p11)(made p23)(made p91)(made p125)(made p127)(made p135)(made p156)(made p159)(made p226)(made p227)(made p251)(made p257)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o183))(shipped o183)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o184
:parameters (?avail ?new-avail - count)
:precondition (and (started o184)(made p167)(made p188)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o184))(shipped o184)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o185
:parameters (?avail ?new-avail - count)
:precondition (and (started o185)(made p53)(made p107)(made p141)(made p152)(made p180)(made p188)(made p210)(made p217)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o185))(shipped o185)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o186
:parameters (?avail ?new-avail - count)
:precondition (and (started o186)(made p136)(made p150)(made p234)(made p260)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o186))(shipped o186)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o187
:parameters (?avail ?new-avail - count)
:precondition (and (started o187)(made p49)(made p55)(made p90)(made p132)(made p264)(made p271)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o187))(shipped o187)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o188
:parameters (?avail ?new-avail - count)
:precondition (and (started o188)(made p17)(made p18)(made p41)(made p61)(made p107)(made p138)(made p173)(made p187)(made p194)(made p267)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o188))(shipped o188)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o189
:parameters (?avail ?new-avail - count)
:precondition (and (started o189)(made p9)(made p37)(made p61)(made p116)(made p138)(made p148)(made p234)(made p276)(made p283)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o189))(shipped o189)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o190
:parameters (?avail ?new-avail - count)
:precondition (and (started o190)(made p8)(made p99)(made p135)(made p139)(made p154)(made p161)(made p204)(made p234)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o190))(shipped o190)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o191
:parameters (?avail ?new-avail - count)
:precondition (and (started o191)(made p57)(made p63)(made p100)(made p194)(made p200)(made p210)(made p212)(made p222)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o191))(shipped o191)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o192
:parameters (?avail ?new-avail - count)
:precondition (and (started o192)(made p8)(made p26)(made p64)(made p99)(made p123)(made p132)(made p147)(made p237)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o192))(shipped o192)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o193
:parameters (?avail ?new-avail - count)
:precondition (and (started o193)(made p4)(made p6)(made p39)(made p58)(made p83)(made p181)(made p188)(made p205)(made p246)(made p257)(made p262)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o193))(shipped o193)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o194
:parameters (?avail ?new-avail - count)
:precondition (and (started o194)(made p53)(made p76)(made p95)(made p109)(made p112)(made p133)(made p147)(made p163)(made p212)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o194))(shipped o194)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o195
:parameters (?avail ?new-avail - count)
:precondition (and (started o195)(made p16)(made p22)(made p111)(made p122)(made p218)(made p230)(made p239)(made p246)(made p249)(made p260)(made p261)(made p288)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o195))(shipped o195)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o196
:parameters (?avail ?new-avail - count)
:precondition (and (started o196)(made p50)(made p54)(made p82)(made p129)(made p134)(made p151)(made p188)(made p215)(made p250)(made p270)(made p276)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o196))(shipped o196)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o197
:parameters (?avail ?new-avail - count)
:precondition (and (started o197)(made p5)(made p50)(made p185)(made p194)(made p258)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o197))(shipped o197)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o198
:parameters (?avail ?new-avail - count)
:precondition (and (started o198)(made p70)(made p89)(made p153)(made p173)(made p175)(made p216)(made p241)(made p263)(made p274)(made p283)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o198))(shipped o198)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o199
:parameters (?avail ?new-avail - count)
:precondition (and (started o199)(made p2)(made p140)(made p145)(made p239)(made p264)(made p279)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o199))(shipped o199)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o200
:parameters (?avail ?new-avail - count)
:precondition (and (started o200)(made p79)(made p85)(made p234)(made p241)(made p267)(made p275)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o200))(shipped o200)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o201
:parameters (?avail ?new-avail - count)
:precondition (and (started o201)(made p21)(made p45)(made p195)(made p202)(made p277)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o201))(shipped o201)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o202
:parameters (?avail ?new-avail - count)
:precondition (and (started o202)(made p25)(made p26)(made p68)(made p111)(made p149)(made p228)(made p232)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o202))(shipped o202)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o203
:parameters (?avail ?new-avail - count)
:precondition (and (started o203)(made p113)(made p129)(made p139)(made p172)(made p219)(made p244)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o203))(shipped o203)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o204
:parameters (?avail ?new-avail - count)
:precondition (and (started o204)(made p20)(made p34)(made p222)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o204))(shipped o204)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o205
:parameters (?avail ?new-avail - count)
:precondition (and (started o205)(made p62)(made p67)(made p105)(made p143)(made p178)(made p193)(made p285)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o205))(shipped o205)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o206
:parameters (?avail ?new-avail - count)
:precondition (and (started o206)(made p21)(made p23)(made p97)(made p169)(made p204)(made p235)(made p240)(made p269)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o206))(shipped o206)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o207
:parameters (?avail ?new-avail - count)
:precondition (and (started o207)(made p50)(made p81)(made p82)(made p96)(made p137)(made p156)(made p172)(made p261)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o207))(shipped o207)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o208
:parameters (?avail ?new-avail - count)
:precondition (and (started o208)(made p20)(made p103)(made p174)(made p244)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o208))(shipped o208)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o209
:parameters (?avail ?new-avail - count)
:precondition (and (started o209)(made p64)(made p84)(made p113)(made p132)(made p275)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o209))(shipped o209)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o210
:parameters (?avail ?new-avail - count)
:precondition (and (started o210)(made p5)(made p7)(made p149)(made p192)(made p215)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o210))(shipped o210)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o211
:parameters (?avail ?new-avail - count)
:precondition (and (started o211)(made p1)(made p66)(made p140)(made p143)(made p225)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o211))(shipped o211)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o212
:parameters (?avail ?new-avail - count)
:precondition (and (started o212)(made p18)(made p29)(made p75)(made p106)(made p111)(made p234)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o212))(shipped o212)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o213
:parameters (?avail ?new-avail - count)
:precondition (and (started o213)(made p54)(made p134)(made p144)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o213))(shipped o213)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o214
:parameters (?avail ?new-avail - count)
:precondition (and (started o214)(made p69)(made p136)(made p152)(made p190)(made p223)(made p231)(made p242)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o214))(shipped o214)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o215
:parameters (?avail ?new-avail - count)
:precondition (and (started o215)(made p23)(made p67)(made p79)(made p82)(made p86)(made p214)(made p262)(made p287)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o215))(shipped o215)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o216
:parameters (?avail ?new-avail - count)
:precondition (and (started o216)(made p3)(made p7)(made p49)(made p75)(made p81)(made p274)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o216))(shipped o216)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o217
:parameters (?avail ?new-avail - count)
:precondition (and (started o217)(made p11)(made p30)(made p41)(made p76)(made p133)(made p138)(made p256)(made p280)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o217))(shipped o217)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o218
:parameters (?avail ?new-avail - count)
:precondition (and (started o218)(made p35)(made p63)(made p144)(made p202)(made p210)(made p272)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o218))(shipped o218)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o219
:parameters (?avail ?new-avail - count)
:precondition (and (started o219)(made p9)(made p42)(made p85)(made p97)(made p186)(made p206)(made p244)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o219))(shipped o219)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o220
:parameters (?avail ?new-avail - count)
:precondition (and (started o220)(made p169)(made p179)(made p206)(made p216)(made p221)(made p272)(made p282)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o220))(shipped o220)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o221
:parameters (?avail ?new-avail - count)
:precondition (and (started o221)(made p10)(made p24)(made p37)(made p50)(made p54)(made p128)(made p186)(made p197)(made p227)(made p281)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o221))(shipped o221)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o222
:parameters (?avail ?new-avail - count)
:precondition (and (started o222)(made p31)(made p98)(made p104)(made p138)(made p195)(made p204)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o222))(shipped o222)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o223
:parameters (?avail ?new-avail - count)
:precondition (and (started o223)(made p57)(made p92)(made p101)(made p130)(made p166)(made p177)(made p235)(made p265)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o223))(shipped o223)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o224
:parameters (?avail ?new-avail - count)
:precondition (and (started o224)(made p1)(made p13)(made p33)(made p58)(made p134)(made p142)(made p149)(made p150)(made p170)(made p189)(made p195)(made p233)(made p235)(made p274)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o224))(shipped o224)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o225
:parameters (?avail ?new-avail - count)
:precondition (and (started o225)(made p27)(made p34)(made p51)(made p107)(made p211)(made p266)(made p276)(made p281)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o225))(shipped o225)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o226
:parameters (?avail ?new-avail - count)
:precondition (and (started o226)(made p5)(made p69)(made p89)(made p245)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o226))(shipped o226)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o227
:parameters (?avail ?new-avail - count)
:precondition (and (started o227)(made p89)(made p205)(made p268)(made p281)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o227))(shipped o227)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o228
:parameters (?avail ?new-avail - count)
:precondition (and (started o228)(made p17)(made p19)(made p20)(made p95)(made p129)(made p142)(made p202)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o228))(shipped o228)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o229
:parameters (?avail ?new-avail - count)
:precondition (and (started o229)(made p48)(made p74)(made p213)(made p243)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o229))(shipped o229)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o230
:parameters (?avail ?new-avail - count)
:precondition (and (started o230)(made p26)(made p131)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o230))(shipped o230)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o231
:parameters (?avail ?new-avail - count)
:precondition (and (started o231)(made p11)(made p77)(made p184)(made p219)(made p243)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o231))(shipped o231)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o232
:parameters (?avail ?new-avail - count)
:precondition (and (started o232)(made p194)(made p227)(made p247)(made p256)(made p281)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o232))(shipped o232)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o233
:parameters (?avail ?new-avail - count)
:precondition (and (started o233)(made p11)(made p152)(made p169)(made p191)(made p196)(made p198)(made p199)(made p226)(made p281)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o233))(shipped o233)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o234
:parameters (?avail ?new-avail - count)
:precondition (and (started o234)(made p10)(made p29)(made p53)(made p84)(made p213)(made p219)(made p228)(made p254)(made p260)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o234))(shipped o234)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o235
:parameters (?avail ?new-avail - count)
:precondition (and (started o235)(made p9)(made p28)(made p84)(made p135)(made p166)(made p274)(made p276)(made p279)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o235))(shipped o235)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o236
:parameters (?avail ?new-avail - count)
:precondition (and (started o236)(made p13)(made p25)(made p92)(made p141)(made p162)(made p170)(made p211)(made p216)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o236))(shipped o236)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o237
:parameters (?avail ?new-avail - count)
:precondition (and (started o237)(made p42)(made p69)(made p111)(made p225)(made p253)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o237))(shipped o237)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o238
:parameters (?avail ?new-avail - count)
:precondition (and (started o238)(made p121)(made p127)(made p138)(made p190)(made p242)(made p278)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o238))(shipped o238)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o239
:parameters (?avail ?new-avail - count)
:precondition (and (started o239)(made p46)(made p87)(made p94)(made p109)(made p205)(made p278)(made p286)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o239))(shipped o239)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o240
:parameters (?avail ?new-avail - count)
:precondition (and (started o240)(made p156)(made p169)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o240))(shipped o240)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o241
:parameters (?avail ?new-avail - count)
:precondition (and (started o241)(made p35)(made p184)(made p256)(made p270)(made p272)(made p279)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o241))(shipped o241)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o242
:parameters (?avail ?new-avail - count)
:precondition (and (started o242)(made p30)(made p51)(made p62)(made p85)(made p107)(made p130)(made p132)(made p262)(made p272)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o242))(shipped o242)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o243
:parameters (?avail ?new-avail - count)
:precondition (and (started o243)(made p11)(made p20)(made p21)(made p50)(made p80)(made p136)(made p172)(made p175)(made p178)(made p180)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o243))(shipped o243)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o244
:parameters (?avail ?new-avail - count)
:precondition (and (started o244)(made p56)(made p232)(made p259)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o244))(shipped o244)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o245
:parameters (?avail ?new-avail - count)
:precondition (and (started o245)(made p32)(made p62)(made p72)(made p106)(made p131)(made p132)(made p183)(made p188)(made p230)(made p254)(made p290)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o245))(shipped o245)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o246
:parameters (?avail ?new-avail - count)
:precondition (and (started o246)(made p10)(made p80)(made p117)(made p123)(made p163)(made p165)(made p173)(made p193)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o246))(shipped o246)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o247
:parameters (?avail ?new-avail - count)
:precondition (and (started o247)(made p53)(made p121)(made p143)(made p159)(made p201)(made p223)(made p247)(made p274)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o247))(shipped o247)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o248
:parameters (?avail ?new-avail - count)
:precondition (and (started o248)(made p96)(made p131)(made p165)(made p239)(made p246)(made p249)(made p277)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o248))(shipped o248)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o249
:parameters (?avail ?new-avail - count)
:precondition (and (started o249)(made p17)(made p90)(made p120)(made p169)(made p223)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o249))(shipped o249)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o250
:parameters (?avail ?new-avail - count)
:precondition (and (started o250)(made p53)(made p119)(made p135)(made p158)(made p181)(made p197)(made p286)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o250))(shipped o250)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o251
:parameters (?avail ?new-avail - count)
:precondition (and (started o251)(made p48)(made p49)(made p89)(made p101)(made p114)(made p150)(made p193)(made p194)(made p224)(made p242)(made p254)(made p271)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o251))(shipped o251)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o252
:parameters (?avail ?new-avail - count)
:precondition (and (started o252)(made p42)(made p68)(made p90)(made p115)(made p140)(made p147)(made p246)(made p274)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o252))(shipped o252)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o253
:parameters (?avail ?new-avail - count)
:precondition (and (started o253)(made p40)(made p93)(made p130)(made p251)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o253))(shipped o253)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o254
:parameters (?avail ?new-avail - count)
:precondition (and (started o254)(made p77)(made p85)(made p190)(made p198)(made p240)(made p242)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o254))(shipped o254)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o255
:parameters (?avail ?new-avail - count)
:precondition (and (started o255)(made p115)(made p156)(made p207)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o255))(shipped o255)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o256
:parameters (?avail ?new-avail - count)
:precondition (and (started o256)(made p79)(made p92)(made p179)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o256))(shipped o256)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o257
:parameters (?avail ?new-avail - count)
:precondition (and (started o257)(made p4)(made p6)(made p100)(made p131)(made p150)(made p165)(made p178)(made p215)(made p228)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o257))(shipped o257)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o258
:parameters (?avail ?new-avail - count)
:precondition (and (started o258)(made p5)(made p81)(made p97)(made p119)(made p138)(made p140)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o258))(shipped o258)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o259
:parameters (?avail ?new-avail - count)
:precondition (and (started o259)(made p28)(made p64)(made p128)(made p147)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o259))(shipped o259)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o260
:parameters (?avail ?new-avail - count)
:precondition (and (started o260)(made p15)(made p28)(made p44)(made p260)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o260))(shipped o260)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o261
:parameters (?avail ?new-avail - count)
:precondition (and (started o261)(made p30)(made p64)(made p76)(made p159)(made p175)(made p264)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o261))(shipped o261)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o262
:parameters (?avail ?new-avail - count)
:precondition (and (started o262)(made p188)(made p193)(made p212)(made p265)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o262))(shipped o262)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o263
:parameters (?avail ?new-avail - count)
:precondition (and (started o263)(made p66)(made p105)(made p116)(made p120)(made p156)(made p269)(made p280)(made p283)(made p288)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o263))(shipped o263)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o264
:parameters (?avail ?new-avail - count)
:precondition (and (started o264)(made p44)(made p72)(made p84)(made p86)(made p93)(made p135)(made p146)(made p157)(made p166)(made p255)(made p266)(made p270)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o264))(shipped o264)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o265
:parameters (?avail ?new-avail - count)
:precondition (and (started o265)(made p4)(made p43)(made p88)(made p110)(made p215)(made p252)(made p278)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o265))(shipped o265)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o266
:parameters (?avail ?new-avail - count)
:precondition (and (started o266)(made p32)(made p125)(made p174)(made p278)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o266))(shipped o266)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o267
:parameters (?avail ?new-avail - count)
:precondition (and (started o267)(made p6)(made p25)(made p189)(made p232)(made p241)(made p253)(made p274)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o267))(shipped o267)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o268
:parameters (?avail ?new-avail - count)
:precondition (and (started o268)(made p26)(made p87)(made p211)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o268))(shipped o268)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o269
:parameters (?avail ?new-avail - count)
:precondition (and (started o269)(made p2)(made p110)(made p119)(made p163)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o269))(shipped o269)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o270
:parameters (?avail ?new-avail - count)
:precondition (and (started o270)(made p26)(made p42)(made p50)(made p104)(made p180)(made p205)(made p210)(made p252)(made p266)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o270))(shipped o270)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o271
:parameters (?avail ?new-avail - count)
:precondition (and (started o271)(made p83)(made p91)(made p119)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o271))(shipped o271)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o272
:parameters (?avail ?new-avail - count)
:precondition (and (started o272)(made p100)(made p142)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o272))(shipped o272)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o273
:parameters (?avail ?new-avail - count)
:precondition (and (started o273)(made p23)(made p27)(made p43)(made p47)(made p70)(made p91)(made p190)(made p227)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o273))(shipped o273)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o274
:parameters (?avail ?new-avail - count)
:precondition (and (started o274)(made p1)(made p24)(made p237)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o274))(shipped o274)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o275
:parameters (?avail ?new-avail - count)
:precondition (and (started o275)(made p38)(made p78)(made p101)(made p118)(made p127)(made p155)(made p174)(made p191)(made p218)(made p243)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o275))(shipped o275)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o276
:parameters (?avail ?new-avail - count)
:precondition (and (started o276)(made p10)(made p15)(made p81)(made p114)(made p178)(made p246)(made p258)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o276))(shipped o276)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o277
:parameters (?avail ?new-avail - count)
:precondition (and (started o277)(made p30)(made p85)(made p212)(made p274)(made p286)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o277))(shipped o277)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o278
:parameters (?avail ?new-avail - count)
:precondition (and (started o278)(made p25)(made p39)(made p125)(made p203)(made p248)(made p253)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o278))(shipped o278)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o279
:parameters (?avail ?new-avail - count)
:precondition (and (started o279)(made p4)(made p18)(made p135)(made p153)(made p196)(made p273)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o279))(shipped o279)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o280
:parameters (?avail ?new-avail - count)
:precondition (and (started o280)(made p19)(made p160)(made p184)(made p213)(made p277)(made p289)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o280))(shipped o280)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o281
:parameters (?avail ?new-avail - count)
:precondition (and (started o281)(made p125)(made p148)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o281))(shipped o281)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o282
:parameters (?avail ?new-avail - count)
:precondition (and (started o282)(made p13)(made p118)(made p140)(made p149)(made p178)(made p220)(made p255)(made p271)(made p287)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o282))(shipped o282)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o283
:parameters (?avail ?new-avail - count)
:precondition (and (started o283)(made p84)(made p129)(made p247)(made p252)(made p262)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o283))(shipped o283)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o284
:parameters (?avail ?new-avail - count)
:precondition (and (started o284)(made p248)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o284))(shipped o284)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o285
:parameters (?avail ?new-avail - count)
:precondition (and (started o285)(made p79)(made p174)(made p180)(made p252)(made p287)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o285))(shipped o285)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o286
:parameters (?avail ?new-avail - count)
:precondition (and (started o286)(made p26)(made p66)(made p105)(made p143)(made p255)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o286))(shipped o286)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o287
:parameters (?avail ?new-avail - count)
:precondition (and (started o287)(made p68)(made p70)(made p96)(made p101)(made p111)(made p113)(made p114)(made p209)(made p213)(made p237)(made p255)(made p267)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o287))(shipped o287)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o288
:parameters (?avail ?new-avail - count)
:precondition (and (started o288)(made p70)(made p97)(made p204)(made p209)(made p244)(made p245)(made p256)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o288))(shipped o288)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o289
:parameters (?avail ?new-avail - count)
:precondition (and (started o289)(made p59)(made p107)(made p111)(made p118)(made p158)(made p203)(made p206)(made p252)(made p268)(made p288)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o289))(shipped o289)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

(:action ship-order-o290
:parameters (?avail ?new-avail - count)
:precondition (and (started o290)(made p8)(made p37)(made p75)(made p83)(made p91)(made p98)(made p111)(made p198)(made p224)(stacks-avail ?avail)(next-count ?avail ?new-avail))
:effect (and (not (started o290))(shipped o290)(not (stacks-avail ?avail))(stacks-avail ?new-avail)))

)
