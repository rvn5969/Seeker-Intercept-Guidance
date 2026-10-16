function build_sfunction
% Compile the same C++ stack used by the benchmark.
here = fileparts(mfilename('fullpath'));
cpp = fileparts(here);
inc = fullfile(cpp,'include'); src = fullfile(cpp,'src');
files = {fullfile(here,'guidance_sfunction.cpp'), fullfile(src,'config.cpp'), ...
         fullfile(src,'estimator.cpp'), fullfile(src,'guidance.cpp'), fullfile(src,'guidance_stack.cpp')};
args = {'-R2018a',['-I' inc]};
if ispc
    args = [args, {'COMPFLAGS="$COMPFLAGS /std:c++20"'}];
else
    args = [args, {'CXXFLAGS=$CXXFLAGS -std=c++20'}];
end
mex(args{:},files{:},'-outdir',here,'-output','guidance_sfunction');
fprintf('Built %s\n',fullfile(here,['guidance_sfunction.' mexext]));
end
