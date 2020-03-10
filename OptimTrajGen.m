classdef OptimTrajGen < TrajGen
    %OPTIMTRAJGEN 이 클래스의 요약 설명 위치
    %   자세한 설명 위치
    
    properties 
        Property1
    end
    
    methods
        function obj = OptimTrajGen(inputArg1,inputArg2)
            %OPTIMTRAJGEN 이 클래스의 인스턴스 생성
            %   자세한 설명 위치
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 이 메서드의 요약 설명 위치
            %   자세한 설명 위치
            outputArg = obj.Property1 + inputArg;
        end
    end
end

