:-op(500,xfy,.).
:-op(600,fx,-).
:-op(550,xfy,\).
:-op(500,xfy,*.*).
.(L, R, L *.* R).
:-op(500,fx,$).

%Server
getSemantics(X):-
	yield([],_),
	schema(X,SemDef,_,_),
	yield(SemDef,_),!.

getGoal(X):-
	yield([],_),
	schema(X,_,[],_),!,false.

getGoal(X):-
	yield([],_),
	schema(X,_,Goal,_),
	yield(Goal,_),!.

getRegulations(X):-
	yield([],_),
	schema(X,_,_,[]),!,false.

getRegulations(X):-
	yield([],_),
	schema(X,_,_,Regulations),
	yield(Regulations,_),!.

%% ALIVE SCHEMA
schema(alive, [
	[inputStream,0,["TRUE"]],
	[rosStream,0,["TRUE"]],
	[memory,0,["TRUE"]],
	[requestStream,0,["TRUE"]],
	[lunar_mission,1,["TRUE"]] ],
	[],
	[] ).

%% ============================================
%% LUNAR EXPLORATION - ROVER 1
%% ============================================

% Sequential exploration 
schema(lunar_mission, [
	[goto_wp(exp00), 1, ["TRUE"]],
	[goto_wp(exp01), 1, [succeeded(goto(exp00))]],
	[goto_wp(exp10), 1, [succeeded(goto(exp01))]],
	[goto_wp(exp11), 1, [succeeded(goto(exp10))]] ],
	[succeeded(goto(exp11))],
	[] ).

% Go to a waypoint: send the command and wait the end 
schema(goto_wp(X), [
	[rosAct(goto(X), rover1_cmd, seed_pdt_rover1/command, 0.5), 1, ["TRUE"]] ],
	[succeeded(goto(X))],
	[] ).

%% ============================================
%% STANDARD CONCRETE SCHEMATA
%% ============================================

schema(q, [[forget(alive),0,["TRUE"]]], [], [] ).
schema(forget(_), [], [], [] ).
schema(remember(_,_), [], [], [] ).
schema(remember(_), [], [], [] ).
schema(inputStream, [], [], [] ).
schema(listing, [], [], [] ).
schema(show(_), [], [], [] ).
schema(show(_,less), [], [], [] ).
schema(gui, [], [], [] ).
schema(requestStream, [], [], [] ).
schema(memory, [], [], [] ).
schema(test, [], [], [] ).
schema(rosStream, [], [], [] ).
schema(joyStream, [], [], [] ).
schema(ltm(_), [], [], [] ).
schema(set(_,_,_), [], [], [] ).
schema(set(_,_), [], [], [] ).
schema(get(_,_,_), [], [], [] ).
schema(get(_,_), [], [], [] ).
schema(timer(_,_,_), [], [], [] ).
schema(compete(_,_,_,_), [], [], [] ).
schema(compete(_,_,_), [], [], [] ).
schema(solve(_,_,_), [], [], [] ).
schema(solve(_,_), [], [], [] ).
schema(rosSolve(_,_,_,_), [], [], [] ).
schema(rosSolve(_,_,_), [], [], [] ).
schema(rosAct(_,_,_,_), [], [], [] ).
schema(rosAct(_,_,_), [], [], [] ).
schema(rosState(_,_), [], [], [] ).
schema(hardSequence(T), [], [hardSequence(T).done], [] ).
schema(softSequence(T), [], [softSequence(T).done], [] ).
schema(dummy(X), [], [dummy(X).goal], [] ).
schema(template(_), [], [], [] ).

%%% LTM utils
absLevel(S,0):-schema(S,[],_,_).
absLevel(S,N):-schema(S,L,_,_), maxAbsLevel(L,LN),N is LN+1.
maxAbsLevel([[SS,_,_]|Rest],RN):-maxAbsLevel(Rest,RN),absLevel(SS,SSN),RN>=SSN.
maxAbsLevel([[SS,_,_]|_],SSN):-absLevel(SS,SSN).
subSchemaList([[SS,_,_]|Rest],[SS|SSRest]):-subSchemaList(Rest,SSRest).
subSchemaList([[SS,_,_]],[SS]).
subSchemaList(S,SSL):-schema(S,List,_,_),subSchemaList(List,SSL).
