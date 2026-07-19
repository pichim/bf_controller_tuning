%==========================================================================
% PLOT_UTILS - Betaflight Controller Analysis Visualization Class
%==========================================================================
% Purpose: 
%   This class provides visualization methods for analyzing quadcopter 
%   flight controller performance and tuning data.
%
% Authors: 
%   Yuri Bianchi
%   Janick Dort
%   Dario Jurietti
%
% Supervisors: 
%   Michael Peter
%   Prof. Dr. Ruprecht Altenburger
%
% Date: 05.06.2026
%==========================================================================

classdef plot_utils

    properties
        data_flight         % Objekt of Typ flight_data
        gyro_tuning         % Objekt of Typ flight_ctrl_tuning
        analysis_flight     % Objekt of Typ flight_analyser 
        angle_tuning
        do_insert_legends    (1,1) logical = true
        linewidth            (1,1) double  = 1.2
        colorOrder = get(0, 'DefaultAxesColorOrder')
        axis_names cell = {'Roll', 'Pitch', 'Yaw'}
        pos_bode double = [0.1514+0.05, 0.5838-0.2, 0.7536, 0.3472+0.2; ...
                           0.1514+0.05, 0.08,       0.7536, 0.2017] 
    
    end
    methods
        function obj = plot_utils(legend)
            % Konstruktor: speichert die Objekte im Plot-Helper
            obj.do_insert_legends      = legend;
        end
        
        % =================================================================
        %  FIGURE FLIGHT DATA
        % =================================================================
        % Plot any Flight Data you want

        function plotFlightData(obj, df, groups, figTitle)

            nSubplots = length(groups);
            fig = figure('Name', figTitle, ...
             'NumberTitle','off');

            if nargin > 3
                sgtitle(figTitle);   % Gesamt-Titel über allen Subplots
            end

            indNames = fieldnames(df.ind);

            for i = 1:nSubplots
                ax = subplot(nSubplots,1,i);
                hold(ax,'on')

                idxList = groups{i}.idx;
                
                % Check if custom legend is provided in the group
                if isfield(groups{i}, 'legend')
                    legendEntries = groups{i}.legend;
                else
                    % Fall back to auto-generating from field names
                    legendEntries = strings(1, length(idxList));
                    for k = 1:length(idxList)
                        idx = idxList(k);
                        for n = 1:length(indNames)
                            if any(df.ind.(indNames{n}) == idx)
                                legendEntries(k) = indNames{n};
                                break
                            end
                        end
                    end
                end
                
                % Plot the data
                for k = 1:length(idxList)
                    idx = idxList(k);
                    plot(ax, df.time, df.data(:, idx), ...
                 'LineWidth', obj.linewidth);
                end

                grid(ax,'on')
                
                % Set x-axis limits to actual data range
                xlim(ax, [df.time(1), df.time(end)]);

                if obj.do_insert_legends
                    legend(ax, legendEntries, 'Interpreter','none','Location','best');
                end

                ylabel(ax, groups{i}.ylabel)

                if i == nSubplots
                    xlabel(ax,'Time [s]');
                end
            end
        end
        
%%
        % =================================================================
        %  FIGURE CONVERT AND EVALUATION TIME
        % =================================================================

        function obj = plot_Eval_Time(obj, time)

            df = obj.data_flight;
        
            % ---- Figure Handling (immer gleiche Figure überschreiben) ----
            fig = findobj('Type','figure','Name','Evaluation Time Flight');
        
            if isempty(fig)
                fig = figure('Name','Evaluation Time Flight', ...
                             'NumberTitle','off');
            else
                figure(fig);
                clf(fig);
            end
        
            % ---- Plot ----
            delta_time_mus = diff(time)*1.0e6;
        
            plot(time(1:end-1), delta_time_mus)

            hold on
            % Draw dashed line at mean value
            mean_val = mean(delta_time_mus);
            yline(mean_val, '--', sprintf('Mean'), ...
            'LineWidth', obj.linewidth, 'Color', 'k', 'LabelHorizontalAlignment', 'left');
            hold off

            grid on
        
            title(sprintf('Mean: %0.2f us, Median: %0.2f us, Std: %0.2f us', ...
                  mean(delta_time_mus), ...
                  median(delta_time_mus), ...
                  std(delta_time_mus)))
        
            xlabel('Time (sec)')
            ylabel('Ts log (us)')
        
            xlim([0, time(end)])
        
            set(findall(fig, 'type', 'line'), ...
                'LineWidth', obj.linewidth)
        end

         %%
        % =================================================================
        %  FIGURE GYRO SPECTRA
        % =================================================================
        % Generates spectral analysis plots for gyro data

        function obj = plot_Gyro_spectra(obj, fa)

            % ---- Figure Handling (Name-basiert) ----
            figName = 'Gyro Spectra';
            fig = findobj('Type','figure','Name', figName);
        
            if isempty(fig)
                fig = figure('Name', figName, 'NumberTitle','off');
            else
                figure(fig);
                clf(fig);
            end
        
            % ---------- Subplot 1: Unfiltered gyro spectra ----------
            ax(1) = subplot(3, 1, 1);
            plot(ax(1), fa.freq_spectra, fa.spectra(:,1:3));
            grid(ax(1),'on');
            ylabel(ax(1),'Gyro (deg/s)');
            set(ax(1),'YScale','log');
            title(ax(1),'Unfiltered gyro magnitude spectra');
            legend(ax(1), {'Roll','Pitch','Yaw'}, 'Location','northeast');
        
            % ---------- Subplot 2: Filtered (ADC) gyro spectra ----------
            ax(2) = subplot(3, 1, 2);
            plot(ax(2), fa.freq_spectra, fa.spectra(:,4:6));
            grid(ax(2),'on');
            ylabel(ax(2),'Gyro (deg/s)');
            set(ax(2),'YScale','log');
            title(ax(2),'Filtered (ADC) gyro magnitude spectra');
        
            if obj.do_insert_legends
                legend(ax(2), {'Roll','Pitch','Yaw'}, 'Location','northeast');
            end
        
            % ---------- Subplot 3: Axis sum spectra ----------
            ax(3) = subplot(3, 1, 3);
            plot(ax(3), fa.freq_spectra, fa.spectra(:,7:9));
            grid(ax(3),'on');
            ylabel(ax(3),'AxisSum');
            xlabel(ax(3),'Frequency (Hz)');
            set(ax(3),'YScale','log');
            title(ax(3),'Axis sum spectra');
            legend(ax(3), {'Roll','Pitch','Yaw'}, 'Location','northeast');
        
            % ---------- Link axes and Nyquist limit ----------
            linkaxes(ax, 'x');
        
            nyq = 1/(2 * fa.Ts_log);
            xlim([0, nyq]);
        
            % Optional uniform y-limits
            try
                ylim(ax, [1e-3 1e1]);
            catch
            end
        
            set(findall(fig, 'type', 'line'), 'LineWidth', obj.linewidth);
        end
        
        
        %%
        % =================================================================
        %  FIGURE GYRO SPECTROGRAMS
        % =================================================================
        % Generates spectogram plots (Thrust and Frequency)

        function obj = plot_Spectogram(obj, fa, num_spectrograms)

            % ---- Figure Handling (Name-basiert) ----
            figName = 'Gyro Spectrograms';
            fig = findobj('Type','figure','Name', figName);
        
            if isempty(fig)
                fig = figure('Name', figName, 'NumberTitle','off');
            else
                figure(fig);
                clf(fig);
            end
        
            sgtitle(fig, 'Gyro Spectrograms')
        
            axes_labels = {'Roll', 'Pitch', 'Yaw'};
            c_lim = [5e-2 3e0];
        
            % --- Unfiltered Spectrograms ---
            for spectrogram_nr = 1:num_spectrograms
        
                ax = subplot(2, 3, spectrogram_nr);
        
                qmesh = pcolor(ax, ...
                    fa.freq_spectogram{spectrogram_nr}, ...
                    fa.throttle_all{spectrogram_nr}, ...
                    fa.spectrogram_unf{spectrogram_nr});
        
                shading(ax, 'flat')
                set(qmesh, 'EdgeColor', 'none');
        
                colormap(ax, 'jet')
                set(ax, 'ColorScale', 'log')
                clim(ax, c_lim)
                ylim(ax, [0 100])
        
                xlabel(ax, 'Frequency (Hz)')
                title(ax, [axes_labels{spectrogram_nr}, ' – without Filter'])
        
                if spectrogram_nr == 1
                    ylabel(ax, 'Throttle (%)')
                end
            end
        
            % --- Filtered Spectrograms ---
            for spectrogram_nr = 1:num_spectrograms
        
                ax = subplot(2, 3, spectrogram_nr + 3);
        
                qmesh = pcolor(ax, ...
                    fa.freq_spectogram{spectrogram_nr}, ...
                    fa.throttle_all{spectrogram_nr}, ...
                    fa.spectrogram_fil{spectrogram_nr});
        
                shading(ax, 'flat')
                set(qmesh, 'EdgeColor', 'none');
        
                colormap(ax, 'jet')
                set(ax, 'ColorScale', 'log')
                clim(ax, c_lim)
                ylim(ax, [0 100])
        
                xlabel(ax, 'Frequency (Hz)')
                title(ax, [axes_labels{spectrogram_nr}, ' – with Filter'])
        
                if spectrogram_nr == 1
                    ylabel(ax, 'Throttle (%)')
                end
            end
        end
        %%
        % =================================================================
        %  FIGURE BODE PLOTS
        % =================================================================
        % Bode plot (Plant and Coherence) for selected axis

        function obj = plot_Bode_Plant(obj, td, ind_ax, customLabel, G)

            axisName = obj.axis_names{ind_ax};

            figName    = sprintf('Bode Plot %s %s - %s', G, customLabel, axisName);
            plantTitle = sprintf('%s (%s) - %s', G, customLabel, axisName);

            % ---- Figure Handling (überschreiben wenn vorhanden) ----
            fig = findobj('Type','figure','Name', figName);

            if isempty(fig)
                fig = figure('Name', figName, 'NumberTitle','off');
            else
                figure(fig);
                clf(fig);
            end

            opt = bode_plot_options('dB', 'linear', 'deg', 'Hz');

            % --- Plant ---
            ax(1) = subplot('Position', obj.pos_bode(1,:));
            if strcmp(G, 'Plant')
                bode(ax(1), td.P{ind_ax}, 'k', td.omega_bode, opt);
            elseif strcmp(G, 'Complementary Sensitivity')
                bode(ax(1), td.T{ind_ax}, 'k', td.omega_bode, opt)
            elseif strcmp(G, 'Controller')
                bode(ax(1), td.C{ind_ax}, 'k', td.omega_bode, opt)
            end

            title(plantTitle);

            

            % --- Coherence ---
            ax(2) = subplot('Position', obj.pos_bode(2,:));
            opt_coh = bode_plot_options('abs', 'linear', 'deg', 'Hz');
            if strcmp(G, 'Plant')
                bodemag(ax(2), td.Coh_P{ind_ax}, td.omega_bode, '-k', opt_coh);
            elseif strcmp(G, 'Complementary Sensitivity')
                bodemag(ax(2), td.Coh_T{ind_ax}, td.omega_bode, '-k', opt_coh);
            elseif strcmp(G, 'Controller')
                bodemag(ax(2), td.Coh_C{ind_ax}, td.omega_bode, '-k', opt_coh);
            end
            title('');  % Remove auto-generated title
            ylabel(ax(2), 'Coherence');
            ylim(ax(2), [0 1.1]);

            linkaxes(ax, 'x');
            set(findall(fig, 'type', 'line'), 'LineWidth', obj.linewidth);
        end
%%
        % =================================================================
        %  FIGURE GANG OF FOUR
        % =================================================================
        function obj = plot_Gang_of_Four(obj, td, label)
        
            if nargin < 3 || isempty(label)
                label = 'Signal';
            end
        
            ind_ax = td.ind_ax;
        
            % ---- Figure Handling (Name-basiert, refresh) ----
            figName = sprintf('Gang of Four %s - %s', label, obj.axis_names{ind_ax});
            fig = findobj('Type','figure','Name', figName);
        
            if isempty(fig)
                fig = figure('Name', figName, 'NumberTitle','off');
            else
                figure(fig);
                clf(fig);
            end
        
            % ---- Bode Options ----
            opt = bode_plot_options('dB', 'linear', 'deg', 'Hz');
    
        
            % =========================
            % --- Tracking T ---
            % =========================
            ax(1) = subplot(2,2,1);
            bodemag(ax(1), td.CL_ana.T, td.CL_ana_new.T, td.T{ind_ax}, opt);
            title('Tracking T');
            if obj.do_insert_legends
                legend('actual','new','measured','Location','best');
            end
        
            % =========================
            % --- Sensitivity S ---
            % =========================
            ax(2) = subplot(2,2,2);
            bodemag(ax(2), td.CL_ana.S, td.CL_ana_new.S, opt);
            title('Sensitivity S')
            if obj.do_insert_legends
                legend('actual','new','Location','northwest');
            end
        
            % =========================
            % --- Controller Effort SC ---
            % =========================
            ax(3) = subplot(2,2,3);
            bodemag(ax(3), td.CL_ana.SC, td.CL_ana_new.SC, opt);
            title('Controller Effort SC')
            if obj.do_insert_legends
                legend('actual','new','Location','northwest');
            end
        
            % =========================
            % --- Compliance SP ---
            % =========================
            ax(4) = subplot(2,2,4);
            bodemag(ax(4), td.CL_ana.SP, td.CL_ana_new.SP, opt);
            title('Compliance SP')
            if obj.do_insert_legends
                legend('actual','new','Location','southwest');
            end
        
            % ---- Formatting ----
            linkaxes(ax,'x');
            xlim(ax(1),[0.4 600]);
        
            set(findall(fig,'type','line'),'LineWidth',obj.linewidth);
        
            sgtitle(sprintf('Gang of Four %s - %s', label, obj.axis_names{ind_ax}));
        end
%%
        % =================================================================
        %  FIGURE STEP RESPONSE
        % =================================================================
        function obj = plot_Step_Compliance(obj, td, label, ylab)
        
            % td    = tuning object (gyro_ctrl_tuning oder angle_ctrl_tuning)
            % label = frei wählbar, z.B. 'Gyro' oder 'Angle'
            if nargin < 3 || isempty(label)
                label = 'Signal';
            end
        
            axName = obj.axis_names{td.ind_ax};
        
            % ---- Figure Handling (Name-basiert, refresh) ----
            figName = sprintf('Step Response %s - %s', label, axName);
            fig = findobj('Type','figure','Name', figName);
        
            if isempty(fig)
                fig = figure('Name', figName, 'NumberTitle','off');
            else
                figure(fig);
                clf(fig);
            end
        
            % ---- Plot 1: Tracking 
            ax(1) = subplot(2,1,1);
            set(ax(1), 'XScale','linear', 'YScale','linear');
            plot(ax(1), td.step_time, td.step_resp_tra);
            grid(ax(1), 'on');
            ylabel(ax(1), ylab);
            title(ax(1), sprintf('Tracking T - %s', axName));
            xlim(ax(1), [0 0.3]);
        
            if obj.do_insert_legends
                legend(ax(1), 'actual calculated', 'new calculated', 'measured', ...
                       'Location', 'best');
            end
        
            ylim(ax(1), 'auto');
            yl = ylim(ax(1));
            ylim(ax(1), [yl(1), yl(2)*1.05]);
                    
            % ---- Plot 2: Compliance ----
            ax(2) = subplot(2,1,2);
            set(ax(2), 'XScale','linear', 'YScale','linear');
            plot(ax(2), td.step_time, td.step_resp_com);
            grid(ax(2), 'on');
            ylabel(ax(2), ylab);
            xlabel(ax(2), 'Time (sec)');
            title(ax(2), 'Compliance SP');
            ylim(ax(2), 'auto');
        
            % ---- Formatting ----
            linkaxes(ax, 'x');
            xlim(ax(2), [0 0.3]);
                    
            set(findall(fig, 'type', 'line'), 'LineWidth', obj.linewidth);
        
            sgtitle(sprintf('Step Response %s - %s', label, axName));
        end

         % =================================================================
        %  FIGURE STEP RESPONSE
        % =================================================================
        function obj = plot_Step_Response(obj, td, label, ylab)
        
            % td    = tuning object (gyro_ctrl_tuning oder angle_ctrl_tuning)
            % label = frei wählbar, z.B. 'Gyro' oder 'Angle'
            if nargin < 3 || isempty(label)
                label = 'Signal';
            end
        
            axName = obj.axis_names{td.ind_ax};
        
            % ---- Figure Handling (Name-basiert, refresh) ----
            figName = sprintf('Step Response %s - %s', label, axName);
            fig = findobj('Type','figure','Name', figName);
        
            if isempty(fig)
                fig = figure('Name', figName, 'NumberTitle','off');
            else
                figure(fig);
                clf(fig);
            end
        
            % ---- Plot 1: Tracking 
            plot(td.step_time, td.step_resp_tra, LineWidth=1.5);
            grid on;
            ylabel(ylab);
            xlim([0 1]);

            title(sprintf('Step Response %s - %s', label, axName), 'FontSize', 15);
            subtitle(sprintf('Tracking T - %s', axName), 'FontSize', 11);
        
            if obj.do_insert_legends
                legend('actual calculated', 'new calculated', 'measured', ...
                       'Location', 'best');
            end
        end
    end
end