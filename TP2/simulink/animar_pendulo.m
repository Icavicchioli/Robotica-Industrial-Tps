function animar_pendulo(out,fps)

if nargin < 2
    fps = 30;
end

t_original = out.tout;

x_original = squeeze(out.simout.signals.values);

if size(x_original,2) ~= 4
    x_original = x_original.';
end

% -------------------------------------------------
% Remuestreo temporal uniforme para el video
% -------------------------------------------------

Ttot = t_original(end) - t_original(1);

Nframes = max(round(Ttot*fps),1);

t = linspace(t_original(1),...
             t_original(end),...
             Nframes);

q1 = interp1(t_original,...
             x_original(:,1),...
             t,...
             'linear');

q2 = interp1(t_original,...
             x_original(:,2),...
             t,...
             'linear');

% -------------------------------------------------

a1 = 0.2;
a2 = 0.2;

x1 = a1*cos(q1);
y1 = a1*sin(q1);

x2 = x1 + a2*cos(q1+q2);
y2 = y1 + a2*sin(q1+q2);

figure('Position',[100 100 560 420])

axis equal
grid on
hold on

L = a1 + a2;

xlim([-L L])
ylim([-L L])

h = plot([0 x1(1) x2(1)],...
         [0 y1(1) y2(1)],...
         '-o',...
         'LineWidth',3,...
         'MarkerSize',8);

traj = plot(x2(1),y2(1),...
            'LineWidth',1.5);

txt = text(-0.35,0.35,'');

% ---------------- VIDEO ----------------

v = VideoWriter('pendulo.mp4','MPEG-4');

v.FrameRate = fps;

open(v);

% ---------------------------------------
set(gcf,'Resize','off')
for k = 1:length(t)

    set(h,...
        'XData',[0 x1(k) x2(k)],...
        'YData',[0 y1(k) y2(k)])

    set(traj,...
        'XData',x2(1:k),...
        'YData',y2(1:k))

    q1deg = rad2deg(q1(k));
    q2deg = rad2deg(q2(k));

    set(txt,...
        'String',...
        sprintf(['t = %.2f s\n' ...
                 'q1 = %.1f°\n' ...
                 'q2 = %.1f°'],...
                 t(k),...
                 q1deg,...
                 q2deg))

    drawnow

    frame = getframe(gcf);

    writeVideo(v,frame);

end

close(v);

close(gcf);

disp('Video generado: pendulo.mp4')

end

