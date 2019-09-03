classdef Kinematics
    properties
       model 
    end
    methods
        %% Construct kinematics with kinematics model configuration
        function obj = Kinematics(m)
           obj.model = m; 
        end
        %% Helper function for unary operations
        function result = unary(~, c, t, f)
            if (c)
                result = t;
            else
                result = f;
            end
        end
        %% Forward Kinematics (FK)
        function [HtRSp, HtLSp, HtRSr, HtLSr, HtRE, HtLE, HtRHy, HtLHy, HtRHr, HtLHr, HtRHp, HtLHp, HtRK, HtLK, HtRAp, HtLAp, HtRAr, HtLAr, HtHy, HtHp] = getFK(this, p)
            % Get FK for arms
            [RSp, RSr, RE] = this.armFK(p, false);
            [LSp, LSr, LE] = this.armFK(p, true);
            % Get FK for legs
            [RHy, RHr, RHp, RK, RAp, RAr] = this.legFK(p, false);
            [LHy, LHr, LHp, LK, LAp, LAr] = this.legFK(p, true);
            % Get FK for Head
            [Hy, Hp] = this.headFK(p);

            % Return FK in servo order
            HtRSp = RSp.tf;
            HtLSp = LSp.tf;
            HtRSr = RSr.tf;
            HtLSr = LSr.tf;
            HtRE = RE.tf;
            HtLE = LE.tf;
            HtRHy = RHy.tf;
            HtLHy = LHy.tf;
            HtRHr = RHr.tf;
            HtLHr = LHr.tf;
            HtRHp = RHp.tf;
            HtLHp = LHp.tf;
            HtRK = RK.tf;
            HtLK = LK.tf;
            HtRAp = RAp.tf;
            HtLAp = LAp.tf;
            HtRAr = RAr.tf;
            HtLAr = LAr.tf;
            HtHy = Hy.tf;
            HtHp = Hp.tf;
        end
        %% FK: Head yaw and head pitch servos
        function [HtHy, HtHp] = headFK(this, p)
            % Alias servo positions for convenience
            headYaw = p(19);
            headPitch = p(20);
            % Traverse to head yaw
            t = Transform3D();
            t = t.translate(this.model.head.neck.POSITION)...
                 .rotateY(-pi/2)...
                 .rotateX(headYaw)...
                 .translateX(this.model.head.neck.LENGTH);
            % Store head yaw
            HtHy = t;
            % Traverse to head pitch
            t = t.rotateY(pi/2)...
                 .rotateY(headPitch)...
                 .translate(this.model.head.NECK_TO_CAMERA)...
                 .rotateY(this.model.head.CAMERA_DECLINATION);
            % Store head pitch
            HtHp = t;
        end
        %% FK: Shoulder pitch, roll and elbow servos
        function [HtSp, HtSr, HtE] = armFK(this, p, isLeft)
            % Alias servo positions for convenience
            shoulderPitch = p(1+isLeft);
            shoulderRoll = p(3+isLeft);
            elbow = p(5+isLeft);
            % Mirror about y if right leg
            mirrorY = this.unary(isLeft, 1.0, -1.0);
            % Traverse to shoulder pitch
            t = Transform3D();
            t = t.translate([this.model.arm.shoulder.OFFSET(1) this.model.arm.DISTANCE_BETWEEN_SHOULDERS*mirrorY/2 this.model.arm.shoulder.OFFSET(2)])...
                .rotateY(shoulderPitch - pi/2)...
                .translate(this.model.arm.shoulder.DIMS .* [1 mirrorY -1]);
            % Store shoulder pitch
            HtSp = t;
            % Traverse to shoulder roll
            t = t.rotateX(shoulderRoll)...
                .translate([this.model.arm.upper_arm.OFFSET(2) mirrorY*this.model.arm.upper_arm.OFFSET(1) -this.model.arm.upper_arm.LENGTH])...
                .rotateY(pi/2);
            % Store shoulder roll
            HtSr = t;
            % Traverse to elbow
            t = t.rotateY(elbow)...
                .translate([this.model.arm.lower_arm.LENGTH mirrorY*this.model.arm.lower_arm.OFFSET(1) -this.model.arm.lower_arm.OFFSET(2)]);
            % Store elbow
            HtE = t;
        end
        %% FK: Hip yaw, hip roll, hip pitch, knee, ankle pitch and ankle roll servos
        function [HtHy, HtHr, HtHp, HtK, HtAp, HtAr] = legFK(this, p, isLeft)
            % Alias servo positions for convenience
            hipYaw      = p(7+isLeft);
            hipRoll     = p(9+isLeft);
            hipPitch    = p(11+isLeft);
            knee        = p(13+isLeft);
            anklePitch  = p(15+isLeft);
            ankleRoll   = p(17+isLeft);
            % Mirror about y if right leg
            mirrorY = this.unary(isLeft, 1.0, -1.0);
            %Traverse to hip yaw
            t = Transform3D(); 
            t = t.translate(this.model.leg.HIP_OFFSET .* [1 mirrorY -1])...
                .rotateY(pi/2)...
                .rotateX(-hipYaw);
            % Store hip yaw
            HtHy = t;
            % Traverse to hip roll
            t = t.rotateZ(hipRoll);
            % Store hip roll
            HtHr = t;
            % Traverse to hip pitch
            t = t.rotateY(hipPitch)...
                .translateX(this.model.leg.UPPER_LEG_LENGTH);
            % Store hip pitch
            HtHp = t;
            % Traverse to knee
            t = t.rotateY(knee)...
                .translateX(this.model.leg.LOWER_LEG_LENGTH);
            % Store knee
            HtK = t;
            % Traverse to ankle pitch
            t = t.rotateY(anklePitch);
            % Store ankle pitch
            HtAp = t;
            % Traverse to ankle roll
            t = t.rotateZ(ankleRoll)...
                .rotateY(-pi/2)...
                .translateZ(-this.model.leg.foot.HEIGHT);
            % Store ankle roll
            HtAr = t;
        end
        %% Inverse Kinematics (IK)
        function [thetaTRSp, thetaTLSp, thetaTRSr, thetaTLSr, thetaTRE, thetaTLE, thetaTRHy, thetaTLHy, thetaTRHr, thetaTLHr, thetaTRHp, thetaTLHp, thetaTRK, thetaTLK, thetaTRAp, thetaTLAp, thetaTRAr, thetaTLAr, thetaTNy, thetaTNp] = getIK(this, rTRh, rTLh, HTRl, HTLl, rCP)
            % Head IK
            [thetaTNy, thetaTNp] = this.headIK(rCP);
            % Arm IK
            [thetaTRSp, thetaTRSr, thetaTRE] = this.armIK(rTRh, false);
            [thetaTLSp, thetaTLSr, thetaTLE] = this.armIK(rTLh, true);
            % Leg IK
            [thetaTRHy, thetaTRHr, thetaTRHp, thetaTRK, thetaTRAp, thetaTRAr] = this.legIK(HTRl.tf, false);
            [thetaTLHy, thetaTLHr, thetaTLHp, thetaTLK, thetaTLAp, thetaTLAr] = this.legIK(HTLl.tf, true);
            % Pack theta values
%             p = [thetaTRSp, thetaTLSp, thetaTRSr, thetaTLSr, thetaTRE, thetaTLE, thetaTRHy, thetaTLHy, thetaTRHr, thetaTLHr, thetaTRHp, thetaTLHp, thetaTRK, thetaTLK, thetaTRAp, thetaTLAp, thetaTRAr, thetaTLAr, thetaTNy, thetaTNp].';
            %p
        end
        %% IK: Head yaw, head pitch
        function [thetaTNy, thetaTNp] = headIK(this, rTt)
            thetaTNy = atan2(rTt(2), rTt(1));
            thetaTNp = atan2(-rTt(3), norm(rTt(1:2)));
        end
        %% IK: Shoulder pitch, shoulder roll, elbow servos
        function [thetaTSp, thetaTSr, thetaTE] = armIK(this, rTt, isLeft)
            % Mirror about y if right leg
            mirrorY = this.unary(isLeft, 1.0, -1.0);
            % Vector from shoulder to torso
            rTS = [this.model.arm.shoulder.OFFSET(1) mirrorY*this.model.arm.DISTANCE_BETWEEN_SHOULDERS/2 this.model.arm.shoulder.OFFSET(2)];
            % Vector from hand (target) to shoulder
            rSt = rTt - rTS;
            % Elbow
            extLength = norm(rSt);
            sqrUpperArm = this.model.arm.upper_arm.LENGTH ^ 2;
            sqrLowerArm = this.model.arm.lower_arm.LENGTH ^ 2;
            sqrExtenArm = extLength ^ 2;
            cosElbow = (sqrUpperArm + sqrLowerArm - sqrExtenArm) / (2 * this.model.arm.upper_arm.LENGTH * this.model.arm.lower_arm.LENGTH);
            thetaTE = -pi + acos(max(min(cosElbow, 1), -1));
            % Shoulder Pitch
            cosPitAngle = (sqrUpperArm + sqrExtenArm - sqrLowerArm) / (2 * this.model.arm.upper_arm.LENGTH * extLength);
            thetaTSp = acos(max(min(cosPitAngle, 1), -1)) + atan2(-rSt(3), rSt(1));
            % Shoulder roll
            % Get negative pitch rotation
            Rp = Transform3D();
            Rp = Rp.rotateY(-thetaTSp);
            % Vector from hand (target) to shoulder without pitch
            rStp = Rp.tf * [rSt 1.0].';
            thetaTSr = atan2(rStp(2), rStp(1));
        end
        %% IK: Hip yaw, hip roll, hip pitch, knee, ankle pitch, ankle roll servos
        function [thetaTHy, thetaTHr, thetaTHp, thetaTK, thetaTAp, thetaTAr] = legIK(this, Htf, isLeft)
            HTt = Transform3D();
            HTt.tf = Htf;
            % Homogeneous transform from target to torso in torso space
            HTt = HTt.translate([0 0 this.model.leg.foot.HEIGHT]);
            % Homogeneous transform from foot to torso in torso space
            HTf = Transform3D();
            % Swizzle for x<->y
            HTf.tf = [0 1 0 0; 1 0 0 0; 0 0 -1 0; 0 0 0 1];
            p = HTf.tf * [HTt.tf(1:3, 4); 1];
            HTt.tf = HTf.tf * HTt.tf * transpose(HTf.tf);
            HTt.tf(1:3, 4) = p(1:3);
            % If left limb, invert x translation
            if ~isLeft
                HTt.tf(1, 4) = HTt.tf(1,4) * -1;
            end
            
            ankleX = HTt.tf(1:3, 1);
            ankleY = HTt.tf(1:3, 2);
            ankleP = HTt.tf(1:3, 4);
            
            % Define target leg as ankle position - hip_offset
            targetLeg = ankleP - [this.model.leg.HIP_OFFSET(2); this.model.leg.HIP_OFFSET(1); this.model.leg.HIP_OFFSET(3)];
            
            legLength = norm(targetLeg);
            maxLegLength = this.model.leg.UPPER_LEG_LENGTH + this.model.leg.LOWER_LEG_LENGTH;
            % Check if leg has passed maximum length
            targetLeg = this.unary(legLength > maxLegLength, targetLeg * maxLegLength / legLength, targetLeg);

            legLength = norm(targetLeg);
            % Alias squared values for convenience
            sqrLength = legLength ^ 2;
            sqrUpperLeg = this.model.leg.UPPER_LEG_LENGTH ^ 2;
            sqrLowerLeg = this.model.leg.LOWER_LEG_LENGTH ^ 2;
            
            cosKnee = (sqrUpperLeg + sqrLowerLeg - sqrLength) / (2 * this.model.leg.UPPER_LEG_LENGTH * this.model.leg.LOWER_LEG_LENGTH);
            thetaTK = acos(max(min(cosKnee, 1), -1));
            
            cosLowerLeg = (sqrLowerLeg + sqrLength - sqrUpperLeg) / (2 * this.model.leg.LOWER_LEG_LENGTH * legLength);
            lowerLeg = acos(max(min(cosLowerLeg, 1), -1));
            
            phi2 = acos(max(min(dot(targetLeg, ankleY) / legLength, 1), -1));
            thetaTAp = lowerLeg + phi2 - pi/2;
            
            unitTargetLeg = targetLeg / legLength;
            
            hipX = cross(ankleY, unitTargetLeg);
            hipXLength = norm(hipX, 2);
            hipX = this.unary(hipXLength > 0, hipX / hipXLength, hipX);
            if hipXLength <= 0
                disp('[ERR] TargetLeg and ankleY parallel.')
            end

            legPlaneTangent = cross(ankleY, hipX);
            thetaTAr = atan2(dot(ankleX, legPlaneTangent), dot(ankleX, hipX));
            
            % Globals
            rx = [1;0;0];
            ry = [0;1;0];
            rz = [0;0;1];
            
            cosZandHipX = dot(rz, hipX);
            
            % If ankle is above waist, swap z component            
            legPlaneGlobalZ = rz - (cosZandHipX * hipX);
            legPlaneGlobalZ = this.unary(dot(unitTargetLeg, rz) < 0, -legPlaneGlobalZ, legPlaneGlobalZ);
            
            legPlaneGlobalZLength = norm(legPlaneGlobalZ, 2);
            legPlaneGlobalZ = this.unary(legPlaneGlobalZLength > 0, legPlaneGlobalZ / legPlaneGlobalZLength, legPlaneGlobalZ);
            
            thetaTHr = acos(max(min(dot(legPlaneGlobalZ, rz), 1), -1));
            thetaTHr = this.unary(cosZandHipX <= 0, thetaTHr, -thetaTHr);

            % Calculate hip pitch
            phi4 = pi - thetaTK - lowerLeg;
            
            thetapiphi2 = sin(pi - phi2);
            unitUpperLeg = unitTargetLeg * (sin(phi2 - phi4) / thetapiphi2) + ankleY * (sin(phi4) / thetapiphi2);
            
            thetaTHp = acos(max(min(dot(legPlaneGlobalZ, unitUpperLeg), 1), -1));
            thetaTHp = this.unary(dot(hipX, cross(unitUpperLeg, legPlaneGlobalZ)) >= 0, thetaTHp, -thetaTHp);
            
            % Calculate hip yaw
            hipXProjected = this.unary(dot(unitTargetLeg, rz) < 0, -hipX, hipX);
            hipXProjected(3) = 0;
            hipXProjected = hipXProjected / norm(hipXProjected, 2);
            
            thetaTHy = acos(dot(hipXProjected, rx));
            thetaTHy = this.unary(dot(hipXProjected, ry) >= 0, thetaTHy, -thetaTHy);            

            % Correct for angle directions
            thetaTHy = -thetaTHy;
            thetaTHp = -thetaTHp;
            thetaTK  = pi - thetaTK;
            thetaTAp = -thetaTAp;
            if ~isLeft
                thetaTHy = thetaTHy * this.model.leg.left_to_right.HIP_YAW;
                thetaTHr = thetaTHr * this.model.leg.left_to_right.HIP_ROLL;
                thetaTHp = thetaTHp * this.model.leg.left_to_right.HIP_PITCH;
                thetaTK  = thetaTK  * this.model.leg.left_to_right.KNEE;
                thetaTAp = thetaTAp * this.model.leg.left_to_right.ANKLE_PITCH;
                thetaTAr = thetaTAr * this.model.leg.left_to_right.ANKLE_ROLL;
            end
            
        end
        
    end
end