function [Pavg, freq, y_axis] = estimate_spectrogram(x, y, window, Noverlap, Nest, resolution, Ts)

    % Todo: check if it is usefull to remove mean here
    % x = x - mean(x);
    % y = y - mean(y);

    Ndata = size(x, 1);

    % find the minimum and maximum values of y and generate linearly spaced vector for the output y-axis
    ymin = min(y);
    ymax = max(y);
    dy = (ymax - ymin) / resolution;
    y_axis = (ymin:dy:ymax-dy).';

    % compute the frequency bins for the output x-axis
    df = 1 / (Nest * Ts);
    freq = (0:df:1/Ts-df).';
    ind = freq <= 1 / (2 * Ts);
    freq = freq(ind);
    Nfreq = length(freq);

    % factor 2 so that the magnitude corresponds to a single sided spectrum
    % 2.3*sin(2*pi*f0*time) <=> sqrt(puu(f0)) = 2.3
    W = sum(window) / Nest / 2;

    Pavg = zeros(resolution, Nfreq);
    Navg = zeros(resolution, 1);

    x_ind_start = 1;
    x_ind_end   = Nest;
    Ndelta      = Nest - Noverlap;

    y_ind_min =  inf;
    y_ind_max = -inf;
    while x_ind_end <= Ndata

        % calculate the start and end indices for the y values
        % y_ind_start = x_ind_start + floor((Nest - Ndelta)/2);
        % y_ind_end   = x_ind_start + floor((Nest + Ndelta)/2) - 1;
        y_ind_start = x_ind_start;
        y_ind_end   = x_ind_end;
        % y_ind_start = x_ind_start;
        % y_ind_end   = x_ind_start + round(0.1*Nest);

        % map y values to indices in the spectrogram resolution
        y_ind = round((y(y_ind_start:y_ind_end) - ymin) / (ymax - ymin) * (resolution - 1)) + 1;

        % % y_ind = [1 9 9 8 5 6 5 7 8].';
        % y_ind_sorted = sort(y_ind);
        % % 1     5     5     6     7     8     8     9     9
        % y_ind_count  = zeros(size(y_ind_sorted));
        % j = 1;
        % for i = 1:length(y_ind_sorted)
        %     if (y_ind_sorted(i) ~= y_ind_sorted(j))
        %         j = j + 1;
        %         y_ind_sorted(j) = y_ind_sorted(i);
        %     end
        %     y_ind_count(j) = y_ind_count(j) + 1;
        % end

        % size(y_ind)
        if min(y_ind) < y_ind_min
            y_ind_min = min(y_ind);
        end
        if max(y_ind) > y_ind_max
            y_ind_max = max(y_ind);
        end

        x_act = x(x_ind_start:x_ind_end);
    
        % Todo: check if it is usefull to remove mean here
        % inp_act = inp_act - mean(inp_act);

        x_act = window .* x_act;

        X = fft(x_act) / (Nest * W);
        Pact = X .* conj(X);

        % accumulate the magnitudes into the spectrogram matrix
        for i = 1:length(y_ind)
            Pavg(y_ind(i), :) = Pavg(y_ind(i), :) + Pact(1:Nfreq).';
            Navg(y_ind(i)) = Navg(y_ind(i)) + 1;
        end

        % Pavg(y_ind, :) = Pavg(y_ind, :) + Pact(1:Nfreq).';
        % Navg(y_ind) = Navg(y_ind) + 1;

        % % accumulate the magnitudes into the spectrogram matrix
        % for i = 1:j
        %     Pavg(y_ind(i), :) = Pavg(y_ind(i), :) + y_ind_count(y_ind(i)) * Pact(1:Nfreq).';
        %     Navg(y_ind(i)) = Navg(y_ind(i)) + y_ind_count(y_ind(i));
        % end

        x_ind_start = x_ind_start + Ndelta;
        x_ind_end   = x_ind_end   + Ndelta;

    end

    Pavg = Pavg(Navg ~= 0,:) ./ Navg(Navg ~= 0);
    y_ind_min
    y_ind_max

end
