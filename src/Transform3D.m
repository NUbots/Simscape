classdef Transform3D
    properties
        tf
    end
    
    methods
        %% Constructor
        function t = Transform3D(x, y, z, roll, pitch, yaw)
            % Start transform as identity
            t.tf = eye(4);
            % If we have enough arguments, translate and rotate as needed
            if nargin >= 6
               t = t.translate([x y z]).rotateZ(yaw).rotateY(pitch).rotateX(roll);
            end
        end
        %% Translation
        function p = translate(this, v)
           this.tf = this.tf * [1 0 0 v(1); 0 1 0 v(2); 0 0 1 v(3); 0 0 0 1];
           p = this;
        end
        function p = translateX(this, x)
           p = this.translate([x 0 0]);
        end
        function p = translateY(this, y)
           p = this.translate([0 y 0]);
        end
        function p = translateZ(this, z)
           p = this.translate([0 0 z]);
        end
        %% Rotation
        function q = rotateX(this, roll)
           r = 180 * roll / pi;
           Rx = [1 0 0 0; 0 cosd(r) -sind(r) 0; 0 sind(r) cosd(r) 0; 0 0 0 1];
           this.tf = this.tf * Rx;
           q = this;
        end
        function q = rotateY(this, pitch)
           p = 180 * pitch / pi;
           Ry = [cosd(p) 0 sind(p) 0; 0 1 0 0; -sind(p) 0 cosd(p) 0; 0 0 0 1];
           this.tf = this.tf * Ry;
           q = this;
        end
        function q = rotateZ(this, yaw)
           y = 180 * yaw / pi;
           Rz = [cosd(y) -sind(y) 0 0; sind(y) cosd(y) 0 0; 0 0 1 0; 0 0 0 1];
           this.tf = this.tf * Rz;
           q = this;
        end
        %% Overload matrix multiplication operation
        function t = mtimes(t1, t2)
           t = Transform3D();
           t.tf = t1.tf * t2.tf;
        end
        %% Overload matrix inverse operation
        function t = inv(this)
           t = Transform3D();
           t.tf = inv(this.tf);
        end
    end
end