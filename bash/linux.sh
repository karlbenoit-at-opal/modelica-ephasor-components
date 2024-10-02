#!/bin/echo Usage: source linux.sh

function build_local() {
    mkdir -p ./linux64
    pushd ./linux64 > /dev/null
    OPENMODELICALIBRARY="${LOCAL_MODELICA_PATH}:${LOCAL_MODELICA_PATH}/MSL4:../sources" ${LOCAL_OPEN_MODELICA_COMPILER} "../sources/build.mos"
    popd > /dev/null
}

function check_local_env() {
    echo Verifying Open Modelica Installation on host ${HOSTNAME} ...
    (
        set +e;
        command -v "${LOCAL_OPEN_MODELICA_COMPILER}"

        if [[ $? -ne 0 ]];
        then
            echo Open Modelica Compiler not found
            echo Expecting omc at: "${LOCAL_OPEN_MODELICA_COMPILER}"
            exit 1
        fi

        echo Found: ${LOCAL_OPEN_MODELICA_COMPILER}
    )
    
    echo Verifying modelica library requirements on localhost ...
    modelica_std_lib="${LOCAL_MODELICA_PATH}/MSL4/Modelica ${MODELICA_VERSION}"
    modelica_opalrt_lib="${LOCAL_MODELICA_PATH}/OpalRT"

    
    (
        set +e;
        (
            set -e;
            [ -d "${modelica_std_lib}" ] && echo Found: $(realpath "${modelica_std_lib}") || false
            [ -d "${modelica_opalrt_lib}" ] && echo Found: $(realpath "${modelica_opalrt_lib}") || false
        )
        
        if [[ $? -ne 0 ]];
        then
            echo Some modelica libraries are missing. 
            echo Make sure the following libraries are present at the following location on the remote target:
            echo "   ${modelica_std_lib}"
            echo "   ${modelica_opalrt_lib}"
            exit 1
        fi
    )

    echo All checks passed on ${HOSTNAME}
}

LOCAL_OPEN_MODELICA_COMPILER="/usr/openmodelica/OpenModelica/bin/omc"
LOCAL_MODELICA_PATH="${HOME}/.modelica"