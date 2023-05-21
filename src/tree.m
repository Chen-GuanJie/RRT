classdef tree < handle

    properties (SetAccess = public)
        children = cell(1, 1) %todo: use neet peer instead of recording children
        parent = zeros(1, 1, 'uint32')
        node_num = 0
        max_nodes = uint32(0)
        new_node = struct
        preorder = zeros(1, 1, 'uint32')
        mapping = zeros(1, 1, 'uint32')
        next_peer = zeros(1, 1, 'uint32')
    end

    methods (Access = public)

        function init(this)
            this.parent = zeros(this.max_nodes, 1, 'uint32');
            this.children = cell(this.max_nodes, 1);
            this.node_num = 0;
            this.new_node.id_parent = uint32(0);
            this.preorder = zeros(1, 1, 'uint32');
            this.preorder(1, 1) = 1;
        end

        function next_id = next_peer_id(this, id)
            %get next peer's id or find parent's if none
            parent_id = this.parent(id, 1);

            if ~parent_id
                next_id = 0;
                return
            end

            peers = this.children{parent_id};
            ind_self = find(peers == id);

            if ind_self < length(peers)
                next_id = peers(ind_self + 1);
            elseif ind_self == length(peers) %is last one
                next_id = this.next_peer_id(parent_id);
            end

        end

        function next_ind = next_preorder_ind(this, id)
            next_id = this.next_peer_id(id);

            if next_id
                next_ind = find(this.preorder == next_id);
            else
                next_ind = length(this.preorder) + 1;
            end

        end

        function insert_node(this)
            this.node_num = this.node_num + 1;
            this.new_node.id = this.node_num;
            parent_id = this.new_node.id_parent;
            this.parent(this.new_node.id, 1) = parent_id;
            this.children{this.new_node.id} = [];

            if parent_id > 0
                this.children{parent_id}(end + 1) = this.new_node.id;
                ind = this.next_preorder_ind(this.new_node.id);
                this.preorder = [this.preorder(1:ind - 1); this.new_node.id; this.preorder(ind:end)]; %insert before parent's next peer
            end

        end

        function delete_node(this, id_remain, id_delete)
            %map old id to new id
            this.mapping(this.node_num, 1) = 0;
            this.mapping(id_remain, 1) = 1:length(id_remain);
            %delete
            this.preorder(ismember(this.preorder, id_delete)) = [];
            this.parent(id_delete) = [];
            this.children(id_delete) = [];
            this.node_num = length(this.parent);
            %map
            this.preorder = this.mapping(this.preorder);
            this.parent(2:end) = this.mapping(this.parent(2:end));

            for i = 1:this.node_num
                this.children{i} = this.children{i}(~ismember(this.children{i}, id_delete));
                this.children{i} = this.mapping(this.children{i});
            end

        end

        function change_parent(this, id, new_parent_id)
            preorder_id = this.preorder(ismember(this.preorder, id), 1); %ordered id

            for i = length(preorder_id):-1:1 %change in inverse order
                self_id = preorder_id(i);
                ind = find(this.preorder == self_id);
                next_ind = this.next_preorder_ind(self_id); %next index of last offspring in preorder array
                insert_ind = this.next_preorder_ind(new_parent_id); %insert index

                if ind < insert_ind % move the sub tree to new place
                    this.preorder = [this.preorder(1:ind - 1); this.preorder(next_ind:insert_ind - 1); this.preorder(ind:next_ind - 1); this.preorder(insert_ind:end)];
                else
                    this.preorder = [this.preorder(1:insert_ind - 1); this.preorder(ind:next_ind - 1); this.preorder(insert_ind:ind - 1); this.preorder(next_ind:end)];
                end

                old_parent_id = this.parent(self_id);
                peer = this.children{old_parent_id};
                this.children{old_parent_id} = peer(~ismember(peer, self_id));
                this.children{new_parent_id}(end + 1) = self_id;
                this.parent(self_id, 1) = new_parent_id;
            end

        end

        function ids = get_ancestor(this, prev)
            ids = [];
            ids(1, 1) = prev;
            ind = 2;
            prev = this.parent(prev, 1);

            while prev > 0
                ids(ind, 1) = prev;
                ind = ind + 1;
                prev = this.parent(prev, 1);
            end

        end

        function offspring = get_offspring(this, id, offspring)

            if nargin == 2
                ind_end = this.next_preorder_ind(id);
                ind_start = find(this.preorder == id) + 1;
                offspring = this.preorder(ind_start:ind_end - 1);
            elseif nargin == 3 %preorder traversal

                if isempty(this.children{id})
                    offspring = [];
                else
                    tmp = this.children{id};
                    offspring = [offspring tmp];

                    for i = 1:length(tmp)
                        offspring = this.get_offspring(tmp(i), offspring);
                    end

                end

            end

        end

    end

end
