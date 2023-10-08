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
        t
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = uav_viewer
            self.body_handle = [];
            [self.Vertices, self.Faces, self.facecolors] = self.define_spacecraft();
            self.plot_initialized = 0;
            self.t = 0;
        end
        %---------------------------
        function self=update(self, state_old, fig, e)
            state = state_old;
            [state(5),state(6),state(4)] = states_with_wind(state(5),state(6),state(4),e,1);
            if self.plot_initialized==0
                fig;
                self=self.drawBody(state(1), state(2), state(3),...
                                   state(7), state(6), state(4));
%                 title('Colliding UAVs')
                xlabel('East (nm)')
                ylabel('North (nm)')
                zlabel('-Down (nm)')
                view(-47,36)  % set the vieew angle for figure                
                axis('equal');
%                 axis(0.1*[-15,15,-15,15,-5,15]);
                hold on
                grid on
                rotate3d on
                self.plot_initialized = 1;
            else
                self.t = self.t + 0.5;
%                 view(-35+self.t,36)
                self=self.drawBody(state(1), state(2), state(3),... 
                                   state(7), state(6), state(4));
%                 axis([state(2)-10,state(2)+10,state(1)-10,state(1)+10,-state(3)-10,-state(3)+10]);
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
            vertices = R*vertices;
            if isempty(self.body_handle)
                self.body_handle = patch('Vertices', vertices', 'Faces', self.Faces,...
                                             'FaceVertexCData',self.facecolors,...
                                             'FaceColor','flat');
            else
                set(self.body_handle,'Vertices',vertices','Faces',self.Faces);
                drawnow
            end
        end 
        %---------------------------
        function pts=rotate(self, pts, phi, theta, psi)
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
            R = R_roll*R_pitch*R_yaw;   % inertial to body
            R = R';  % body to inertial
            % rotate vertices
            pts = R*pts;
        end
        %---------------------------
        % translate vertices by pn, pe, pd
        function pts = translate(self, pts, pn, pe, pd)
            pts = pts + repmat([pn;pe;pd],1,size(pts,2));
        end
        %---------------------------
        function [V, F, colors] = define_spacecraft(self)
            % Define the vertices (physical location of vertices)
%             V = 0.05*[...
%                 1    1    0;... % point 1
%                 1   -1    0;... % point 2
%                 -1   -1    0;... % point 3
%                 -1    1    0;... % point 4
%                 1    1   -2;... % point 5
%                 1   -1   -2;... % point 6
%                 -1   -1   -2;... % point 7
%                 -1    1   -2;... % point 8
%                 1.5  1.5  0;... % point 9
%                 1.5 -1.5  0;... % point 10
%                 -1.5 -1.5  0;... % point 11
%                 -1.5  1.5  0;... % point 12
%             ]';
            
            V = 0.01*[...
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
%             F = [...
%                     1, 2,  3,  4;...  % front
%                     4, 3,  7,  8;...  % back
%                     1, 5,  8,  4;...  % right 
%                     2, 6,  7,  3;...  % left
%                     5, 6,  7,  8;...  % top
%                     9, 10, 11, 12;... % bottom
%                     ];
            F = [...
                    1, 2,  3,  4;...  % Body
                    2, 6,  8,  5;...  % Wing Left
                    4, 7,  9,  5;...  % Wing Right 
                    10, 11,  12,  13;...  % Tail
                    ];

            % define colors for each face    
%             myred = [1, 0, 0];
%             mygreen = [0, 1, 0];
%             myblue = [0, 0, 1];
%             myyellow = [1, 1, 0];
%             mycyan = [0, 1, 1];
            myred = [1, 0, 0];
            mygreen = [0, 1, 0];
            myblue = [0, 0, 1];
            myyellow = [1, 1, 0];
            mycyan = [0, 1, 1];


%             colors = [...
%                 myyellow;... % front
%                 myblue;...   % back
%                 myblue;...   % right
%                 myblue;...   % left
%                 myblue;...   % top
%                 mygreen;...  % bottom
%                 ];
            colors = [...
                myred;...  % Body
                myblue;... % Wing Left
                myblue;... % Wing Right
                mygreen];...% Tail

        end
    end
end