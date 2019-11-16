function qplot7(t, q)
    if nargin < 2
        q = t;
        t = (1:numrows(q))';
    end
    %clf
    hold on
    plot(t, q(:,1:3))
    plot(t, q(:,4:7), '--')
    grid on
    xlabel('Time (s)')
    ylabel('Joint coordinates (rad,m)')
    legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7');
    hold off

    xlim([t(1), t(end)]);

