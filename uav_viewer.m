classdef uav_viewer < handle
    %
    %    Create spacecraft animation
    %
    %--------------------------------
    properties
        body_handle
    	Vertices
    	Faces
    	facecolors
        plot_initialized
        scale
        t
        focus_uav
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = uav_viewer(focus_uav)
            self.body_handle = [];
            self.scale = 0.02;
            self.focus_uav = focus_uav;
            [self.Vertices, self.Faces, self.facecolors] = self.define_spacecraft();
            self.plot_initialized = 0;
            self.t = 0;
        end
        %---------------------------
        function self = update(self, time, state_old, uav_n, e, scenario_num)
            state = state_old;
            [~, ~, state(7)] = states_with_wind(state(4), state(6), state(7), e, 1);
            self.t = time;
            [ts, azs, els, zoomins, centers] = camera_configs(scenario_num, state);

            if self.focus_uav == 0
                zoomin = zoomins(1);
                az = azs(1);
                el = els(1);
                center = centers(:, 1);
            else
                if self.t <= ts(2)
                    mode = 1;
                elseif self.t <= ts(3)
                    mode = 2;
                elseif self.t <= ts(4)
                    mode = 3;
                else
                    mode = 1;
                end
                [zoomin, center, az, el] = self.camera_motion(ts, zoomins, centers, azs, els, mode);
            end

            if self.plot_initialized == 0
                % self.scale = 0.065 * zoomin - 0.04;
                [self.Vertices, self.Faces, self.facecolors] = self.define_spacecraft();
                self = self.drawBody(state(1), state(2), state(3), ...
                                   state(5), state(6), state(7));
                xlabel('$\rho_e$ (nm)', 'Interpreter', 'latex')
                ylabel('$\rho_n$ (nm)', 'Interpreter', 'latex')
                zlabel('$h$ (nm)', 'Interpreter', 'latex')
                if self.focus_uav == uav_n
                    axis([-zoomin + center(1), zoomin + center(1), ...
                          -zoomin + center(2), zoomin + center(2), ...
                          0, zoomin + center(3)]);
                elseif self.focus_uav == 0
                    axis([-zoomin, zoomin, ...
                          -zoomin, zoomin, ...
                          0, zoomin]);
                end
                view(az, el)  % set the vieew angle for figure                
                hold on
                grid on
                title(gca, sprintf("Time: %.2f sec", self.t));
                self.plot_initialized = 1;
                set(gca, 'FontName', 'times')
            else
                % self.scale = 0.065 * zoomin - 0.04;
                [self.Vertices, self.Faces, self.facecolors] = self.define_spacecraft();
                view(az, el)  % set the vieew angle for figure                
                self = self.drawBody(state(1), state(2), state(3), ... 
                                   state(5), state(6), state(7));
                if self.focus_uav == uav_n
                    axis([-zoomin + center(1), zoomin + center(1), ...
                          -zoomin + center(2), zoomin + center(2), ...
                          0, zoomin + center(3)]);
                elseif self.focus_uav == 0
                    axis([-zoomin, zoomin, ...
                          -zoomin, zoomin, ...
                          0, zoomin]);
                end
                title(gca, sprintf("Time: %.2f sec", self.t));
                set(gca, 'FontName', 'times')
            end
        end
        %---------------------------
        function self = drawBody(self, pn, pe, pd, phi, theta, psi)
            vertices = self.rotate(self.Vertices, phi, theta, psi);   % rotate rigid body  
            vertices = self.translate(vertices, pn, pe, pd);     % translate after rotation
            % transform vertices from NED to ENU (for matlab rendering)
            R = [...
                0, 1, 0;...
                1, 0, 0;...
                0, 0, -1;...
                ];
            vertices = R * vertices;
            if isempty(self.body_handle)
                self.body_handle = patch('Vertices', vertices', 'Faces', self.Faces, ...
                                             'FaceVertexCData', self.facecolors, ...
                                             'FaceColor', 'flat');
            else
                set(self.body_handle, 'Vertices', vertices', 'Faces', self.Faces);
                drawnow
            end
        end 
        %---------------------------
        function pts = rotate(self, pts, phi, theta, psi)
            % define rotation matrix (right handed)
            R_roll = [...
                        1, 0, 0;...
                        0, cos(phi), sin(phi);...
                        0, -sin(phi), cos(phi)];
            R_pitch = [...
                        cos(theta), 0, -sin(theta);...
                        0, 1, 0;...
                        sin(theta), 0, cos(theta)];
            R_yaw = [...
                        cos(psi), sin(psi), 0;...
                        -sin(psi), cos(psi), 0;...
                        0, 0, 1];
            R = R_roll * R_pitch * R_yaw;   % inertial to body
            R = R';  % body to inertial
            % rotate vertices
            pts = R * pts;
        end
        %---------------------------
        % translate vertices by pn, pe, pd
        function pts = translate(self, pts, pn, pe, pd)
            pts = pts + repmat([pn;pe;pd], 1, size(pts, 2));
        end
        %---------------------------
        function [V, F, colors] = define_spacecraft(self)
            % Define the vertices (physical location of vertices)          
            V = self.scale * [...
                 1.5     0      0;... % point 1
                 0      -0.75   0;... % point 2
                -3.25   0      0;... % point 3
                 0       0.75   0;... % point 4
                -0.75    0      0;... % point 5
                -0.75   -3.5   -0.15;... % point 6
                -0.75    3.5   -0.15;... % point 7
                -1      -3.5   -0.15;... % point 8
                -1       3.5   -0.15;... % point 9
                -2.50    0      0;... % point 10
                -3.15    0      0;... % point 11
                -3.35    0     -0.75;... % point 12
                -3.00    0     -0.75;... % point 13
            ]';

            % define faces as a list of vertices numbered above
            F = [...
                    1, 2,  3,  4;...  % Body
                    2, 6,  8,  5;...  % Wing Left
                    4, 7,  9,  5;...  % Wing Right 
                    10, 11,  12,  13;...  % Tail
                    ];

            % define colors for each face    
            myred = [1, 0, 0];
            mygreen = [0, 1, 0];
            myblue = [0, 0, 1];
            myyellow = [1, 1, 0];
            mycyan = [0, 1, 1];

            colors = [...
                myred;...  % Body
                myblue;... % Wing Left
                myblue;... % Wing Right
                mygreen];...% Tail

        end

        function [zoomin, center, az, el] = camera_motion(self, ts, zoomins, centers, azs, els, i)
            slope_zoom = (zoomins(i+1) - zoomins(i)) / (ts(i+1) - ts(i));
            slope_center = (centers(:, i+1) - centers(:, i)) / (ts(i+1) - ts(i));            
            slope_az = (azs(i+1) - azs(i)) / (ts(i+1) - ts(i));
            slope_el = (els(i+1) - els(i)) / (ts(i+1) - ts(i));            
            zoomin = zoomins(i) + slope_zoom * (self.t - ts(i));
            center = centers(:, i) + slope_center * (self.t - ts(i));
            az = azs(i) + slope_az * (self.t - ts(i));
            el = els(i) + slope_el * (self.t - ts(i));
        end
    end
end