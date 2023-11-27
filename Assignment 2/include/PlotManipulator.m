function [] = PlotConfigurationDirectKinematic(numberOfSteps, qi, qf, geom_model, linkType, n)
%%% PlotConfigurationDirectKinematic function

% Inputs:
% numberOfSteps : number fo steps to pass from qi to qf
% qi : initial robot configuration
% qf : final robot configuration
% geom_model : tree of frames
% linkType : specifies two possible link types: Rotational, Prismatic
% n : plot number (it is useful to print via a for loop)

% Output:
% the figure with the robot and its movements from qi to qf
    
    numberOfLinks = length(linkType);

     % For each single link we want to separate the path in n steps.
    for i = 1:numberOfLinks
        qsteps(:,i) = linspace(qi(i),qf(i),numberOfSteps)';
    end
    
    % Plotting the movement of the manipulator.
    figure
    title("Manipulator's Motion: ", n);
    xlabel('x')
    ylabel('y')
    zlabel('z')
    grid on
    axis equal
    hold on

    % Defining the point of view of the view
    azimuth = 50;
    elevation = 25;
    view(azimuth,elevation)
    
    % Plotting all the frames through a for loop.
    for i = 1:numberOfSteps
        
        % Initialazing the vector matrix including the base (0; 0; 0).
        bRij = zeros(3,numberOfLinks+1); 
        
        q = qsteps(i,1:numberOfLinks)';
        disp(q)

        % Filling the bitei matrix using the function GetDirectGeometry().
        biTei = GetDirectGeometry(q, geom_model, linkType);
        
        % Extrapolating basic vector with respect to the base from biTei.
        % taking into account the presence of the base (0,0,0).
        for j = 1:numberOfLinks
            bRij(:,j+1) = GetBasicVectorWrtBase(biTei,j);
        end
        
        % plotting the joints
        for j = 2:numberOfLinks+1  
           plot3(bRij(1,j),bRij(2,j),bRij(3,j),'k.','MarkerSize',15)   
        end
    
        % plotting the lines connecting all the joints. 
        line(bRij(1,:),bRij(2,:),bRij(3,:),'LineWidth',3,'Color','cyan')
    
        % plotting the base in (0,0,0).
        plot3(0,0,0,'red.','MarkerSize', 30)
    
        hold on
        getframe;
        
        if i < numberOfSteps 
            cla();
        end
        
    end    
end
